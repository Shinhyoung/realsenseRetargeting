"""run_d455.bat 런처 파일 생성기"""
import os

bat_path = os.path.expanduser(r"~\run_d455.bat")

content = r"""@echo off
echo [D455] d455_env 가상환경 활성화 중...
call "C:\Users\ADMIN\anaconda3\Scripts\activate.bat" d455_env
if errorlevel 1 (
    echo [오류] 가상환경 활성화 실패 - anaconda 경로를 확인하세요.
    pause
    exit /b 1
)
echo [D455] d455_env 활성화 완료
echo [D455] 프로그램 시작 중...
python "c:\0.shinhyoung\Project\realsenseRetargeting\d455_pose3d_osc.py"
pause
"""

with open(bat_path, "w", encoding="utf-8", newline="\r\n") as f:
    f.write(content)

print(f"런처 파일 생성 완료: {bat_path}")
print("이 파일을 더블클릭하면 프로그램이 실행됩니다.")
