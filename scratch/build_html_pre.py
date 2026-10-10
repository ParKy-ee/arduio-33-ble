Import("env")
from pathlib import Path
import runpy

project = Path(env.subst("$PROJECT_DIR"))
runpy.run_path(str(project / "scratch" / "build_html.py"), run_name="__main__")
