all:
	g++ -std=c++17 -w -Ilib -Isrc src/Main.cpp hidapi.dll -shared -o ringcon_driver.dll
clean:
	del ringcon_driver.dll