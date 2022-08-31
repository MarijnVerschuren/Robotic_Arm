from distutils.core import setup, Extension



setup (
	   name = "py_lib",
	   version = "0.1",
	   author = "Marijn",
	   description = """module based on C++ compiled with swig""",
	   ext_modules = [
		   Extension(
				"_py_lib",
				sources=[
					"int\\py_lib.cpp",
					"src\\py_lib.cpp"
				]
		   )
	   ],
	   py_modules = ["py_lib"],
	   swig_version="swigwin-4.0.2",
)