// 깊이 카메라 원본(float32 미터)을 디스크로 내리는 상주 구독자 (Stage 5, DECISIONS 028 예정).
//
// ── 왜 이게 필요한가 ─────────────────────────────────────────────────────────
// RGB 는 카메라 <save> 태그가 PNG 를 떨궈서 파이썬(ML venv)이 바로 읽는다. 깊이도 <save> 가
// 먹기는 하는데(실측) **8비트 RGB 로 정규화된 시각화**라 거리값이 아니다 — 고유값 192개.
// 우리가 고치려는 오차가 3~5cm 인데 8비트 시각화로는 잴 수가 없다.
// 원본 토픽(ignition.msgs.Image, R_FLOAT32, 미터)을 받으려면 ign-transport 를 직접 물어야 하고,
// `ign topic -e` 로 텍스트 덤프하면 1280x720 실수 배열이라 비현실적이다.
// → ww_cmd 와 같은 방식: 디스커버리 한 번 하고 상주하며 원본 바이트를 파일로 내린다.
//
// ── 파일 형식 (자기서술) ─────────────────────────────────────────────────────
//   [uint32 width][uint32 height][float32 data...]   little-endian, 행 우선
//   numpy 로: w,h = np.fromfile(f, np.uint32, 2); d = np.fromfile(f, np.float32, offset=8).reshape(h,w)
//
// ── RGB 프레임과의 정합 ──────────────────────────────────────────────────────
// 파일명을 메시지 헤더의 seq 로 쓴다 → RGB `down_cam0_8.png` 에 `8.bin` 이 대응한다.
// 두 센서가 같은 주기로 같이 시작하므로 seq 가 맞물린다. 시각으로 맞추면 5Hz 에서 최대 200ms
// (0.2m/s 로 4cm) 어긋날 수 있는데, 그건 우리가 고치려는 오차와 같은 크기라 못 쓴다.
//
// 빌드: make ww-depth      실행: build/ww_depth --topics /robot/depth,/robot/depth1 --out artifacts
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <ignition/msgs/image.pb.h>
#include <ignition/transport/Node.hh>

namespace {

std::string OutRoot = "artifacts";
std::atomic<int> Frames{0};

// 헤더 data 에서 key 로 값 찾기 (seq 를 파일명에 쓴다)
std::string HeaderValue(const ignition::msgs::Image &msg, const std::string &key) {
  for (int i = 0; i < msg.header().data_size(); ++i) {
    const auto &d = msg.header().data(i);
    if (d.key() == key && d.value_size() > 0) return d.value(0);
  }
  return "";
}

class Sink {
 public:
  explicit Sink(std::string dir) : dir_(std::move(dir)) {}

  void OnImage(const ignition::msgs::Image &msg) {
    const uint32_t w = msg.width(), h = msg.height();
    const std::string &data = msg.data();
    // R_FLOAT32 가정. 크기가 안 맞으면 조용히 버리지 말고 알린다(형식이 바뀌면 즉시 드러나게).
    if (data.size() != static_cast<size_t>(w) * h * 4) {
      static bool warned = false;
      if (!warned) {
        std::fprintf(stderr, "W %s: data %zu != w*h*4 (%u x %u) — float32 아님?\n",
                     dir_.c_str(), data.size(), w, h);
        warned = true;
      }
      return;
    }
    std::string seq = HeaderValue(msg, "seq");
    if (seq.empty()) seq = std::to_string(n_++);

    const std::string path = dir_ + "/" + seq + ".bin";
    const std::string tmp = path + ".tmp";
    FILE *f = std::fopen(tmp.c_str(), "wb");
    if (!f) return;
    std::fwrite(&w, sizeof(w), 1, f);
    std::fwrite(&h, sizeof(h), 1, f);
    std::fwrite(data.data(), 1, data.size(), f);
    std::fclose(f);
    std::rename(tmp.c_str(), path.c_str());   // 원자적 — 반쯤 쓰인 파일을 파이썬이 안 읽게
    Frames++;
  }

 private:
  std::string dir_;
  int n_ = 0;
};

}  // namespace

int main(int argc, char **argv) {
  std::vector<std::string> topics{"/robot/depth"};
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--topics" && i + 1 < argc) {
      topics.clear();
      std::string s = argv[++i], cur;
      for (char c : s) {
        if (c == ',') { if (!cur.empty()) topics.push_back(cur); cur.clear(); }
        else cur += c;
      }
      if (!cur.empty()) topics.push_back(cur);
    } else if (a == "--out" && i + 1 < argc) {
      OutRoot = argv[++i];
    }
  }

  ignition::transport::Node node;
  std::vector<Sink *> sinks;
  for (size_t i = 0; i < topics.size(); ++i) {
    std::string dir = OutRoot + "/depth" + std::to_string(i);
    std::string mk = "mkdir -p '" + dir + "'";
    if (std::system(mk.c_str()) != 0) { std::fprintf(stderr, "E mkdir %s\n", dir.c_str()); return 1; }
    auto *s = new Sink(dir);
    sinks.push_back(s);
    if (!node.Subscribe<ignition::msgs::Image>(
            topics[i], [s](const ignition::msgs::Image &m) { s->OnImage(m); })) {
      std::fprintf(stderr, "E subscribe %s\n", topics[i].c_str());
      return 1;
    }
    std::fprintf(stderr, "S %s -> %s\n", topics[i].c_str(), dir.c_str());
  }
  std::fprintf(stdout, "R ww_depth ready (%zu cam)\n", topics.size());
  std::fflush(stdout);
  ignition::transport::waitForShutdown();
  return 0;
}
