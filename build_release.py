"""
Release 빌드 스크립트
d455_pose3d_osc.py → exe (PyInstaller onedir)
모델 파일과 필요 DLL을 Release 폴더에 모아줌
"""
import subprocess
import shutil
import os

ROOT = os.path.dirname(os.path.abspath(__file__))
RELEASE_DIR = os.path.join(ROOT, "Release")
MODEL_SRC = r"c:\0.shinhyoung\Project\realsenseTest\pose_landmarker_lite.task"

# PyInstaller 실행
cmd = [
    "pyinstaller",
    "--noconfirm",
    "--onedir",
    "--console",
    "--name", "D455_PoseOSC",
    "--distpath", RELEASE_DIR,
    "--workpath", os.path.join(ROOT, "build_temp"),
    "--specpath", os.path.join(ROOT, "build_temp"),
    # 모델 파일 번들링 (exe와 같은 폴더에 배치)
    "--add-data", f"{MODEL_SRC};.",
    # mediapipe 데이터 포함
    "--collect-data", "mediapipe",
    "--hidden-import", "mediapipe",
    "--hidden-import", "mediapipe.tasks",
    "--hidden-import", "mediapipe.tasks.python",
    "--hidden-import", "mediapipe.tasks.python.vision",
    os.path.join(ROOT, "d455_pose3d_osc_release.py"),
]

print("[빌드] PyInstaller 실행 중...")
subprocess.run(cmd, check=True)

print(f"\n[완료] Release 빌드 완료: {os.path.join(RELEASE_DIR, 'D455_PoseOSC')}")
print("  → D455_PoseOSC.exe 를 실행하세요")
