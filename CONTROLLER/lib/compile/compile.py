import os


run = lambda x: os.system(x)
clear_console = lambda: os.system("cls" if os.name in ["nt", "dos"] else "clear")

compile_folder = os.path.dirname(os.path.abspath(__file__))
root_folder = os.path.dirname(compile_folder)
if __name__ == "__main__":
	run(rf"echo y|del {compile_folder}\int\*.*")
	run(rf"echo y|rmdir {compile_folder}\int\Release /S")
	run(rf"echo y|del {compile_folder}\bin\*.*")

	run(rf"swig -c++ -python -outdir {compile_folder}/int -o {compile_folder}/int/py_lib.cpp {compile_folder}/src/py_lib.i")
	run(rf"python {compile_folder}/setup.py build_ext --include-dirs {compile_folder}/src --build-lib {compile_folder}/bin --build-temp {compile_folder}/int --swig-cpp")

	run(rf"echo F|xcopy {compile_folder}\bin\*.pyd {root_folder}\_py_lib.pyd /y")
	run(rf"echo F|xcopy {compile_folder}\int\py_lib.py {root_folder}\py_lib_wrap.py /y")

	#clear_console()