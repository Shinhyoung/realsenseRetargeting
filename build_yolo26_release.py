"""
YOLO26 Release 빌드 스크립트
d455_yolo26_osc.py → exe (PyInstaller onedir)
YOLO 모델, CUDA DLL, pyrealsense2 등 모든 의존성을 Release 폴더에 패키징
"""
import subprocess
import shutil
import os
import sys
import glob

ROOT = os.path.dirname(os.path.abspath(__file__))
RELEASE_DIR = os.path.join(ROOT, "Release")
BUILD_TEMP = os.path.join(ROOT, "build_temp")
APP_NAME = "D455_YOLO26_PoseOSC"
DIST_DIR = os.path.join(RELEASE_DIR, APP_NAME)

# YOLO 모델 파일
YOLO_MODELS = [
    os.path.join(ROOT, "yolo26n-pose.pt"),
    os.path.join(ROOT, "yolo26m-pose.pt"),
]

# conda 환경 경로
CONDA_ENV = os.path.join(os.environ.get("CONDA_PREFIX", ""), "")
if not CONDA_ENV or not os.path.isdir(CONDA_ENV):
    CONDA_ENV = r"C:\Users\ADMIN\anaconda3\envs\d455_env"

SITE_PACKAGES = os.path.join(CONDA_ENV, "lib", "site-packages")

# ── pyrealsense2 DLL 경로 ──
RS2_DIR = os.path.join(SITE_PACKAGES, "pyrealsense2")

# ── 소스 파일 ──
SOURCE_FILE = os.path.join(ROOT, "d455_yolo26_osc.py")


def find_cuda_dlls():
    """CUDA 런타임 DLL 경로 수집"""
    cuda_path = os.environ.get("CUDA_PATH", r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8")
    cuda_bin = os.path.join(cuda_path, "bin")
    dlls = []
    if os.path.isdir(cuda_bin):
        # 핵심 CUDA 런타임 DLL만 포함
        patterns = [
            "cudart64_*.dll",
            "cublas64_*.dll",
            "cublasLt64_*.dll",
            "cufft64_*.dll",
            "curand64_*.dll",
            "cusparse64_*.dll",
            "cudnn*.dll",
            "nvrtc64_*.dll",
        ]
        for pat in patterns:
            dlls.extend(glob.glob(os.path.join(cuda_bin, pat)))
    return dlls


def build():
    print(f"[빌드] {APP_NAME} Release 빌드 시작")
    print(f"  소스: {SOURCE_FILE}")
    print(f"  출력: {DIST_DIR}")
    print()

    # 모델 파일 확인
    for m in YOLO_MODELS:
        if not os.path.isfile(m):
            print(f"[오류] 모델 파일 없음: {m}")
            print("  → python d455_yolo26_osc.py 를 먼저 실행하면 자동 다운로드됩니다")
            sys.exit(1)

    # ── PyInstaller 실행 ──
    add_data = []

    # pyrealsense2 전체 패키지 (pyd + dll)
    add_data.append(f"--add-data={RS2_DIR};pyrealsense2")

    # YOLO 모델 파일 (exe와 같은 폴더에 배치)
    for m in YOLO_MODELS:
        add_data.append(f"--add-data={m};.")

    cmd = [
        "pyinstaller",
        "--noconfirm",
        "--onefile",
        "--console",
        "--name", APP_NAME,
        "--distpath", RELEASE_DIR,
        "--workpath", BUILD_TEMP,
        "--specpath", BUILD_TEMP,
        # ultralytics 데이터 (YAML 설정 등)
        "--collect-data", "ultralytics",
        # hidden imports
        "--hidden-import", "ultralytics",
        "--hidden-import", "ultralytics.nn",
        "--hidden-import", "ultralytics.nn.tasks",
        "--hidden-import", "ultralytics.trackers",
        "--hidden-import", "ultralytics.trackers.byte_tracker",
        "--hidden-import", "torch",
        "--hidden-import", "torchvision",
        "--hidden-import", "pyrealsense2",
        "--hidden-import", "pythonosc",
        "--hidden-import", "pythonosc.udp_client",
        "--hidden-import", "pythonosc.osc_bundle_builder",
        "--hidden-import", "pythonosc.osc_message_builder",
        "--hidden-import", "cv2",
        "--hidden-import", "numpy",
        "--hidden-import", "tkinter",
        "--hidden-import", "PIL",
        "--hidden-import", "yaml",
        "--hidden-import", "requests",
        "--hidden-import", "tqdm",
        "--hidden-import", "psutil",
        "--hidden-import", "thop",
    ] + add_data + [
        SOURCE_FILE,
    ]

    print("[빌드] PyInstaller 실행 중... (수 분 소요)")
    print(f"  명령: pyinstaller --onedir --name {APP_NAME}")
    print()

    result = subprocess.run(cmd)
    if result.returncode != 0:
        print("[오류] PyInstaller 빌드 실패")
        sys.exit(1)

    # ── 추가 파일 복사 ──
    print()
    print("[복사] 추가 파일 복사 중...")

    # CUDA DLL 복사
    cuda_dlls = find_cuda_dlls()
    if cuda_dlls:
        print(f"  CUDA DLL {len(cuda_dlls)}개 복사")
        for dll in cuda_dlls:
            dst = os.path.join(DIST_DIR, os.path.basename(dll))
            if not os.path.exists(dst):
                shutil.copy2(dll, dst)
    else:
        print("  [경고] CUDA DLL을 찾을 수 없음 — GPU 모드 사용 불가할 수 있음")

    # YOLO 모델 파일을 exe 옆에도 복사 (YOLO는 cwd 기준으로 모델을 찾음)
    for m in YOLO_MODELS:
        dst = os.path.join(DIST_DIR, os.path.basename(m))
        shutil.copy2(m, dst)
        print(f"  모델 복사: {os.path.basename(m)}")

    # pyrealsense2 DLL 추가 확인 (상위 폴더의 realsense2.dll 등)
    rs2_parent = os.path.dirname(RS2_DIR)
    for f in glob.glob(os.path.join(rs2_parent, "realsense2*.dll")):
        dst = os.path.join(DIST_DIR, os.path.basename(f))
        if not os.path.exists(dst):
            shutil.copy2(f, dst)
            print(f"  복사: {os.path.basename(f)}")

    # torch DLL 확인 (누락된 것 보완)
    torch_lib = os.path.join(SITE_PACKAGES, "torch", "lib")
    if os.path.isdir(torch_lib):
        torch_dlls = glob.glob(os.path.join(torch_lib, "*.dll"))
        copied = 0
        for dll in torch_dlls:
            dst = os.path.join(DIST_DIR, os.path.basename(dll))
            if not os.path.exists(dst):
                shutil.copy2(dll, dst)
                copied += 1
        if copied:
            print(f"  torch DLL {copied}개 추가 복사")

    # ── 결과 확인 ──
    exe_path = os.path.join(DIST_DIR, f"{APP_NAME}.exe")
    if os.path.isfile(exe_path):
        # 폴더 크기 계산
        total_size = 0
        for dirpath, dirnames, filenames in os.walk(DIST_DIR):
            for f in filenames:
                total_size += os.path.getsize(os.path.join(dirpath, f))
        size_mb = total_size / (1024 * 1024)

        print()
        print("=" * 60)
        print(f"[완료] Release 빌드 성공!")
        print(f"  경로: {DIST_DIR}")
        print(f"  실행: {exe_path}")
        print(f"  크기: {size_mb:.0f} MB")
        print()
        print("  포함된 파일:")
        print(f"    - {APP_NAME}.exe (메인 실행 파일)")
        print(f"    - yolo26n-pose.pt (Nano 모델)")
        print(f"    - yolo26m-pose.pt (Medium 모델)")
        print(f"    - CUDA DLL ({len(cuda_dlls)}개)")
        print(f"    - pyrealsense2, torch, ultralytics 등 의존성")
        print()
        print("  실행 방법:")
        print(f"    {exe_path}")
        print("    (conda 환경 불필요)")
        print("=" * 60)
    else:
        print("[오류] exe 파일이 생성되지 않았습니다")
        sys.exit(1)


if __name__ == "__main__":
    build()
