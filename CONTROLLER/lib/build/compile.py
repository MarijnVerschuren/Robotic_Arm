import sys, os


if __name__ == "__main__":
	if "-dll" in sys.argv:
		print(os.system("gcc -c dll/com.cpp -o int/com.o"))
		print(os.system("gcc -shared -o bin/com.dll -Wl,--out-implib,dll/libtstdll.a int/com.o"))