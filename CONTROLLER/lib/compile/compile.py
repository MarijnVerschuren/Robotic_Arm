import sys, os



compile_folder = os.path.dirname(__file__)
root_folder = os.path.dirname(compile_folder)

compile_flags = [
	"-O2",
	"-Wall",
	"-shared",
	"-std=c++11",
	"-fPIC",
	"$(python3 -m pybind11 --includes)"
]

files = [
	f"{compile_folder}/src/com.cpp",
	f"{compile_folder}/src/bind.cpp"
]

output = f"{compile_folder}/bin/py_lib$(python3-config --extension-suffix)"

if __name__ == "__main__":
	"""
	if "-dll" in sys.argv:
		print(os.system("gcc -c dll/com.cpp -o int/com.o"))
		print(os.system("gcc -shared -o bin/com.dll -Wl,--out-implib,dll/libtstdll.a int/com.o"))
	"""
	if "-help" in sys.argv:
		print(
			"[-force-clean]:\tdelete all previous build files",
			sep="\n", end="\n\n"
		)
		exit(0)
	if "-force-clean" in sys.argv:
		os.system(f"rm {compile_folder}/bin/*")

	os.system(f"c++ {' '.join(compile_flags)} {' '.join(files)} -o {output}")
	os.system(f"cp {compile_folder}/bin/*.so {root_folder}/py_lib.so")