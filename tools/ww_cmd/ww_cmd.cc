// ww_cmd — 주행 중 폐루프 제어를 가능하게 하는 상주 명령/상태 프로세스 (Stage 4-3).
//
// ── 왜 이게 있어야 하나 (실측) ─────────────────────────────────────────────
// `ign topic -p` 는 발행 한 번에 **1.055초**가 걸린다 (이 컴퓨터에서 5회 측정, 중앙값).
// 프로세스 기동 + 디스커버리 + 광고 + 종료가 매번 반복되기 때문이다.
// 0.25 m/s 로 달리는 로봇에겐 명령 하나에 26cm 다. 성공 허용오차가 2cm 이므로
// **CLI 로는 주행 중 제어가 원천적으로 불가능하다.**
//
// Stage 4-2(스탬핑)가 이 함정을 안 밟은 건 로봇이 서 있었기 때문이다 — 정지 상태에서는
// 1초 지연이 무해하다. 주행이 붙는 순간 지연이 곧 위치 오차가 된다.
//
// 그래서 디스커버리를 한 번만 하고 계속 살아 있는 프로세스를 둔다. 파이프로 말한다.
//
// ── 멀티툴 (DECISIONS 020) ────────────────────────────────────────────────
// 점 타격 툴이 N 개다(기본 3). 각 툴은 독립 Y 캐리지 + 독립 Z. 명령·상태 모두 인덱스로
// 가른다. N 은 --n-tools 로 받는다(하네스가 garden_geometry.Portal.n_tools 를 넘긴다).
//
// ── 왜 `ign topic -e` 로 상태를 못 받나 ──────────────────────────────────
// 파이프로 리다이렉트하면 stdio 가 블록버퍼링(4KB)으로 바뀌어, 50Hz odom 이 버퍼가
// 찰 때까지 최대 수백 ms 밀린다. 제어 루프에 들어가면 그 지연이 그대로 오차다.
// 여기서는 줄마다 flush 해서 지연을 없앤다.
//
// ── 의도적으로 안 하는 것: 지상진실 ──────────────────────────────────────
// 이 프로세스는 `dynamic_pose/info`(지상진실)를 **구독하지 않는다.** 제어기가 GT 를
// 물리적으로 볼 수 없어야 "제어는 오도메트리로, 채점은 지상진실로"가 구조로 강제된다.
// GT 는 별도 프로세스가 받아 사후 채점에만 쓴다. import 한 줄로 뚫리는 규율은 규율이 아니다.
//
// ── 프로토콜 (줄 단위 텍스트) ────────────────────────────────────────────
//   stdin  : "v <lin_x> <ang_z>"      전진/회전 속도 명령
//            "carriage <i> <pos>"     툴 i 의 Y 캐리지 목표 [m] (밴드 중심 기준 상대)
//            "tool <i> <pos>"         툴 i 의 Z 도구 목표 [m] (0=접힘, 음수=하강)
//            "q"                      종료
//   stdout : "R <topic> ..."                          구독/광고 준비 완료
//            "O <simt> <x> <y> <yaw> <vx> <wz>"        오도메트리
//            "I <simt> <roll> <pitch> <yaw>"           IMU 자세 (월드에 imu-system 있을 때만)
//            "J <simt> <c0..c{N-1}> <t0..t{N-1}>"      관절 achieved 위치 (캐리지 N, 툴 N)
//            "E <message>"                             오류
//
// 빌드:  make build/ww_cmd     (g++ + pkg-config. colcon 아님 — src/ 는 ROS 패키지 전용)
// 실행:  ./scripts/env.sh build/ww_cmd [--world <name>] [--model <name>] [--n-tools <N>]

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/model.pb.h>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>

namespace {

std::mutex g_out;
int g_n_tools = 3;  // --n-tools 로 덮어쓴다. OnJoints 콜백이 참조.

// 한 줄 = 한 flush. 파이프 상대가 즉시 읽을 수 있어야 제어 지연이 안 생긴다.
void Emit(const std::string &line) {
  std::lock_guard<std::mutex> lock(g_out);
  std::cout << line << std::endl;
}

double StampSeconds(const ignition::msgs::Header &header) {
  if (!header.has_stamp()) return 0.0;
  return header.stamp().sec() + header.stamp().nsec() * 1e-9;
}

// 쿼터니언 → yaw. roll/pitch 는 주행 채점에 안 쓰므로 yaw 만 낸다(지상진실 쪽에서 따로 봄).
double Yaw(double x, double y, double z, double w) {
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (n > 0.0) { x /= n; y /= n; z /= n; w /= n; }
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void OnOdom(const ignition::msgs::Odometry &msg) {
  const auto &p = msg.pose().position();
  const auto &q = msg.pose().orientation();
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(6);
  os << "O " << StampSeconds(msg.header()) << ' ' << p.x() << ' ' << p.y() << ' '
     << Yaw(q.x(), q.y(), q.z(), q.w()) << ' '
     << msg.twist().linear().x() << ' ' << msg.twist().angular().z();
  Emit(os.str());
}

// IMU 자세 (실제 온보드 센서 — 지상진실이 아니다).
//
// ── 왜 필요한가 (diag_uturn 실측) ────────────────────────────────────────
// 이 로봇은 조향축 없는 **4륜 고정 = 스키드 스티어**다. 제자리로 돌면 네 바퀴가 옆으로
// 긁히는데(scrub), DiffDrive 오도메트리는 바퀴 회전만 적분하므로 그 미끄러짐을 못 본다.
// 실측: 90° 회전 명령에서 odom 은 89.9° 라 보고했고 지상진실은 **63.6°** 였다(오차 26°).
// 두 번 돌면 46°, 재진입 위치가 2.6m 틀어진다 — 두둑 사이 U턴이 odom 만으로는 불가능하다.
// 방위를 되잡을 수 있는 온보드 센서는 IMU 뿐이라 여기에 붙인다.
//
// 주의: gz IMU 의 orientation 은 노이즈 없는 참자세에서 계산된다(적분 안 함). 실물 잔차는
// 제어하는 쪽에서 얹는다(025 와 같은 처리) — 여기서는 센서를 있는 그대로 흘린다.
void OnImu(const ignition::msgs::IMU &msg) {
  const auto &q = msg.orientation();
  const double x = q.x(), y = q.y(), z = q.z(), w = q.w();
  const double sinr = 2.0 * (w * x + y * z), cosr = 1.0 - 2.0 * (x * x + y * y);
  double sinp = 2.0 * (w * y - z * x);
  sinp = sinp > 1.0 ? 1.0 : (sinp < -1.0 ? -1.0 : sinp);
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(6);
  os << "I " << StampSeconds(msg.header()) << ' ' << std::atan2(sinr, cosr) << ' '
     << std::asin(sinp) << ' ' << Yaw(x, y, z, w);
  Emit(os.str());
}

// joint_state 는 Model 메시지로 온다. N 개 캐리지·툴 프리즘 관절을 인덱스별로 뽑는다.
// achieved(sim 이 보고하는 실제 도달 위치)이지 명령이 아니다 — 4-2 가 이 값으로 채점했다.
void OnJoints(const ignition::msgs::Model &msg) {
  std::vector<double> carriage(g_n_tools, std::nan(""));
  std::vector<double> tool(g_n_tools, std::nan(""));
  bool any = false;
  for (const auto &joint : msg.joint()) {
    if (!joint.has_axis1()) continue;
    int idx = -1;
    if (std::sscanf(joint.name().c_str(), "carriage%d_joint", &idx) == 1 &&
        idx >= 0 && idx < g_n_tools) {
      carriage[idx] = joint.axis1().position();
      any = true;
    } else if (std::sscanf(joint.name().c_str(), "tool%d_joint", &idx) == 1 &&
               idx >= 0 && idx < g_n_tools) {
      tool[idx] = joint.axis1().position();
      any = true;
    }
  }
  if (!any) return;
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(6);
  os << "J " << StampSeconds(msg.header());
  for (double c : carriage) os << ' ' << c;
  for (double t : tool) os << ' ' << t;
  Emit(os.str());
}

}  // namespace

int main(int argc, char **argv) {
  std::string world = "robot_row";
  std::string model = "weedwatch";
  for (int i = 1; i < argc - 1; ++i) {
    const std::string arg = argv[i];
    if (arg == "--world") world = argv[++i];
    else if (arg == "--model") model = argv[++i];
    else if (arg == "--n-tools") g_n_tools = std::atoi(argv[++i]);
  }
  if (g_n_tools < 1) g_n_tools = 1;

  ignition::transport::Node node;

  const std::string odom_topic = "/odometry";
  const std::string joint_topic = "/world/" + world + "/model/" + model + "/joint_state";

  if (!node.Subscribe(odom_topic, OnOdom)) {
    Emit("E odometry 구독 실패: " + odom_topic);
    return 1;
  }
  if (!node.Subscribe(joint_topic, OnJoints)) {
    Emit("E joint_state 구독 실패: " + joint_topic);
    return 1;
  }
  // IMU 는 월드에 imu-system 플러그인이 있어야 발행된다. 없는 월드(row·strike 등)에서도
  // 나머지 제어는 그대로 돌아야 하므로 구독 실패를 치명으로 보지 않는다 — 대신 I 줄이 안 온다.
  const std::string imu_topic = "/robot/imu";
  if (!node.Subscribe(imu_topic, OnImu)) {
    Emit("E imu 구독 실패(주행엔 무해, 회전 제어엔 필요): " + imu_topic);
  }

  auto cmd_vel = node.Advertise<ignition::msgs::Twist>("/cmd_vel");
  if (!cmd_vel) { Emit("E 발행자 광고 실패 (/cmd_vel)"); return 1; }

  // N 개 캐리지·툴 명령 발행자. 인덱스로 접근.
  std::vector<ignition::transport::Node::Publisher> carriage_cmd, tool_cmd;
  std::string advertised = "/cmd_vel";
  for (int i = 0; i < g_n_tools; ++i) {
    const std::string ct = "/carriage" + std::to_string(i) + "_cmd";
    const std::string tt = "/tool" + std::to_string(i) + "_cmd";
    auto cp = node.Advertise<ignition::msgs::Double>(ct);
    auto tp = node.Advertise<ignition::msgs::Double>(tt);
    if (!cp || !tp) { Emit("E 발행자 광고 실패: " + ct + " / " + tt); return 1; }
    carriage_cmd.push_back(std::move(cp));
    tool_cmd.push_back(std::move(tp));
    advertised += " " + ct + " " + tt;
  }

  // 하네스가 이 줄을 보고 "명령 경로가 살아 있다"를 확인한 뒤 주행을 시작한다.
  // 디스커버리 레이스를 위치 오차로 위장시키지 않기 위한 신호다.
  Emit("R " + odom_topic + " " + joint_topic + " " + advertised);

  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream is(line);
    std::string verb;
    if (!(is >> verb)) continue;

    if (verb == "q") break;

    if (verb == "v") {
      double lin = 0.0, ang = 0.0;
      if (!(is >> lin >> ang)) { Emit("E v 인자 부족: " + line); continue; }
      ignition::msgs::Twist msg;
      msg.mutable_linear()->set_x(lin);
      msg.mutable_angular()->set_z(ang);
      cmd_vel.Publish(msg);
      continue;
    }

    if (verb == "carriage" || verb == "tool") {
      int idx = 0;
      double pos = 0.0;
      if (!(is >> idx >> pos)) { Emit("E " + verb + " 인자 부족 (i pos): " + line); continue; }
      if (idx < 0 || idx >= g_n_tools) { Emit("E " + verb + " 인덱스 범위밖: " + line); continue; }
      ignition::msgs::Double msg;
      msg.set_data(pos);
      (verb == "carriage" ? carriage_cmd : tool_cmd)[idx].Publish(msg);
      continue;
    }

    Emit("E 알 수 없는 명령: " + line);
  }

  // 종료 전 정지 명령. 마지막 cmd_vel 이 계속 유지되는 Fortress DiffDrive 특성(cmd_timeout
  // 없음, assert_drive 에서 실측) 때문에, 안 세우고 죽으면 로봇이 계속 달린다.
  ignition::msgs::Twist stop;
  stop.mutable_linear()->set_x(0.0);
  stop.mutable_angular()->set_z(0.0);
  cmd_vel.Publish(stop);
  return 0;
}
