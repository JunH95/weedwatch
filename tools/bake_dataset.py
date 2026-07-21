#!/usr/bin/env python3
"""데이터셋 bake — 여러 시드로 CropCraft 를 돌려 학습/평가 세트를 쌓는다 (Stage 3-2a).

한 시드 = 한 정원(레이아웃). 데이터 다양성은 시드에서 온다. train/eval 시드는 겹치지 않는다
(configs/{train,eval}_seeds.txt, 범위 분리). eval 은 보호 대상 — train 에 절대 섞지 마라.

각 시드를 CropCraft 로 렌더(models/render 에 staging) → models/dataset/<split>/{images,masks}
로 seed<N>_frame_<NNNN> 이름으로 이동한다. 이미 구운 시드는 건너뛴다(idempotent):
그래서 시드를 추가하고 다시 돌리면 새 시드만 렌더된다(증분 bake, 이전 렌더 재사용).

⚠️ "bake" 는 데이터셋 렌더(구축)지 학습이 아니다. 학습은 별도(Stage 3-2c, torch).

사용:
  ./scripts/env.sh python3 tools/bake_dataset.py train configs/train_seeds.txt
  ./scripts/env.sh python3 tools/bake_dataset.py eval  configs/eval_seeds.txt
"""
from __future__ import annotations

import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

WW = Path(__file__).resolve().parents[1]
BASE_CFG = WW / "configs" / "train_garden.yaml"     # 시드만 바꿔가며 재사용하는 템플릿
STAGING = WW / "models" / "render"                  # cropcraft.sh 의 기본 출력(스테이징)
CROPCRAFT_SH = WW / "scripts" / "cropcraft.sh"


def seeds_from(path: Path) -> list[int]:
    out = []
    for line in Path(path).read_text().splitlines():
        line = line.split("#")[0].strip()
        if line:
            out.append(int(line))
    if len(out) != len(set(out)):
        raise SystemExit(f"{path}: 시드에 중복이 있다")
    return out


def make_seed_cfg(seed: int, tmpdir: str) -> Path:
    """train_garden.yaml 을 복사하고 random_seed 만 바꾼 임시 config 를 만든다."""
    text = BASE_CFG.read_text()
    new, n = re.subn(r"(?m)^(\s*random_seed:\s*)\d+", rf"\g<1>{seed}", text)
    if n != 1:
        raise SystemExit(f"random_seed 라인을 정확히 1개 못 찾음(찾음={n}) — {BASE_CFG}")
    p = Path(tmpdir) / f"seed_{seed}.yaml"
    p.write_text(new)
    return p


def already_baked(split_dir: Path, seed: int) -> bool:
    imgs = list((split_dir / "images").glob(f"seed{seed}_*.jpg"))
    msks = list((split_dir / "masks").glob(f"seed{seed}_*.png"))
    return len(imgs) > 0 and len(imgs) == len(msks)


def bake_seed(cfg_path: Path, split_dir: Path, seed: int) -> int:
    # 스테이징을 비우고(이전 시드 잔여 방지) 렌더한다.
    for sub in ("images", "masks"):
        d = STAGING / sub
        if d.exists():
            shutil.rmtree(d)
    subprocess.run([str(CROPCRAFT_SH), str(cfg_path)], check=True, cwd=str(WW))
    imgs = sorted((STAGING / "images").glob("*.jpg"))
    msks = sorted((STAGING / "masks").glob("*.png"))
    if not imgs or len(imgs) != len(msks):
        raise SystemExit(f"시드 {seed}: 스테이징 렌더 이상 (img {len(imgs)} / mask {len(msks)})")
    for sub in ("images", "masks"):
        (split_dir / sub).mkdir(parents=True, exist_ok=True)
    for src in imgs:
        shutil.move(str(src), str(split_dir / "images" / f"seed{seed}_{src.name}"))
    for src in msks:
        shutil.move(str(src), str(split_dir / "masks" / f"seed{seed}_{src.name}"))
    return len(imgs)


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("사용: bake_dataset.py <train|eval> <seedfile>")
    split, seedfile = sys.argv[1], sys.argv[2]
    if split not in ("train", "eval"):
        raise SystemExit("split 은 train 또는 eval")

    split_dir = WW / "models" / "dataset" / split
    seeds = seeds_from(Path(seedfile))
    print(f"=== bake {split}: 시드 {len(seeds)}개 → {split_dir} ===", flush=True)

    baked = skipped = frames = 0
    with tempfile.TemporaryDirectory() as tmp:
        for i, seed in enumerate(seeds, 1):
            if already_baked(split_dir, seed):
                skipped += 1
                print(f"  [{i}/{len(seeds)}] seed {seed}: 이미 있음 — 건너뜀", flush=True)
                continue
            print(f"  [{i}/{len(seeds)}] seed {seed}: 렌더 …", flush=True)
            cfg = make_seed_cfg(seed, tmp)
            n = bake_seed(cfg, split_dir, seed)
            baked += 1
            frames += n
            print(f"      +{n}장", flush=True)

    print(f"=== {split} 완료: {baked} 시드 새로 구움({frames}장), {skipped} 건너뜀 ===", flush=True)


if __name__ == "__main__":
    main()
