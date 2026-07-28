#!/usr/bin/env python3
"""새 밭이 물리로 성립하는지 단언 (Tier 2, 렌더 없음) — `make field-check FIELD=dev`.

명세와 생성물은 산수로 확인했다(test_field_spec·test_ridge_varied). 남은 물음은 물리다:

  1. 월드가 뜨고 DART 가 안 터지는가 (굽은 두둑 + 흙덩이 + 경사를 한꺼번에 얹었다)
  2. 로봇이 **스폰 자세에서 두둑을 걸터탄 채 서 있는가** — 폭이 ±5cm 변하면 여유 11cm/쪽 중
     8.7cm 가 잠식된다(042). 서지도 못하면 그 밭은 주행을 논할 수 없다.
  3. 가만히 뒀을 때 굴러떨어지거나 진동하지 않는가 (경사 3° 에서 미끄러짐)

주행·U턴은 다음 단계다. 여기서는 **밭 위에 로봇을 놓을 수 있나**까지만 본다.
"""
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
ENV = str(WW / "scripts" / "env.sh")
sys.path.insert(0, str(WW / "tools"))
from assert_drive import gt_samples  # noqa: E402
from field_spec import get  # noqa: E402
from garden_geometry import Garden  # noqa: E402

G = Garden()


class Fail(Exception):
    pass


def kill(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except (ProcessLookupError, AttributeError):
        pass


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "dev"
    spec = get(name)
    world = WW / "worlds" / f"field_{name}.sdf"
    if not world.exists():
        raise Fail(f"{world} 없음 — make worlds/field_{name}.sdf 로 생성")

    print(f"=== 밭 '{name}' 물리 단언 (렌더 없음) ===")
    print(f"    두둑 {spec.n_beds}×{spec.bed_length}m ({spec.area_m2:.1f}m²) · "
          f"폭±{spec.width_var*100:.0f} 높이±{spec.height_var*100:.0f} 사행±{spec.meander*100:.0f}cm · "
          f"흙덩이 {sum(len(spec.clods(b)) for b in range(spec.n_beds))}개 · 경사 {spec.cross_slope_deg}°\n")

    subprocess.run(["pkill", "-f", "[i]gn gazebo"], capture_output=True)
    time.sleep(0.5)
    gt_topic = f"/world/field_{name}/dynamic_pose/info"
    log = open(f"/tmp/ww_field_{name}.log", "w")
    sim = subprocess.Popen([ENV, "ign", "gazebo", "-s", "-r", "--iterations", "20000", str(world)],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    gtsub = None
    try:
        deadline = time.time() + 30
        while time.time() < deadline:
            t = subprocess.run([ENV, "ign", "topic", "-l"], capture_output=True, text=True).stdout
            if gt_topic in t:
                break
            time.sleep(0.5)
        else:
            raise Fail("월드가 안 떴다 — 로그 확인")
        gf = open(f"/tmp/ww_field_{name}_gt.log", "w")
        gtsub = subprocess.Popen([ENV, "ign", "topic", "-e", "-t", gt_topic],
                                 stdout=gf, stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(8)                     # 스폰 안착 + 정지 관찰
    finally:
        kill(gtsub); time.sleep(0.3); kill(sim)
        try:
            sim.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim.pid), signal.SIGKILL)
        log.close()

    gt = gt_samples(Path(f"/tmp/ww_field_{name}_gt.log").read_text(errors="ignore"))
    if len(gt) < 20:
        raise Fail(f"지상진실 샘플 부족({len(gt)}) — 물리가 안 돌았다")

    settle = gt[len(gt) // 2:]            # 뒷 절반 = 안착 후
    ys = [s[2] for s in settle]
    zs = [s[3] for s in settle]
    rolls = [math.degrees(s[4]) for s in settle]
    pitches = [math.degrees(s[5]) for s in settle]
    cy = spec.bed_centers[0]
    y_err = max(abs(y - cy) for y in ys) * 100
    z_drift = (max(zs) - min(zs)) * 100
    roll_pp = max(rolls) - min(rolls)
    tilt = max(abs(r) for r in rolls)

    print(f"  스폰 후 정지 관찰 ({len(settle)} 샘플)")
    print(f"  {'두둑 중심에서 y':<20}{y_err:>7.1f} cm  (여유 {G.furrow_width/2*100 + 0:.0f}cm/쪽 안이어야)")
    print(f"  {'z 표류(가라앉음)':<20}{z_drift:>7.1f} cm")
    print(f"  {'기울기 roll':<20}{tilt:>7.1f} °   (밭 경사 {spec.cross_slope_deg}°)")
    print(f"  {'roll 진동 p2p':<20}{roll_pp:>7.1f} °   (발산하면 DART 가 터진 것)")

    if any(math.isnan(v) for v in (y_err, z_drift, roll_pp)):
        raise Fail("NaN — 물리가 터졌다")
    if y_err > 11.0:
        raise Fail(f"로봇이 두둑에서 {y_err:.1f}cm 벗어났다 — 걸터타기 실패")
    if z_drift > 5.0:
        raise Fail(f"z 가 {z_drift:.1f}cm 표류 — 가라앉거나 튀고 있다")
    if roll_pp > 5.0:
        raise Fail(f"roll 진동 {roll_pp:.1f}° — 안 정착한다")
    if tilt > spec.cross_slope_deg + 5.0:
        raise Fail(f"기울기 {tilt:.1f}° 가 밭 경사({spec.cross_slope_deg}°)보다 과하다 — 뭔가 탔다")

    print(f"\n[통과] 밭 '{name}' 위에 로봇이 선다. 다음: 이 밭에서 주행·U턴·타격 재측정.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Fail as e:
        print(f"\n[실패] {e}", file=sys.stderr)
        sys.exit(1)
