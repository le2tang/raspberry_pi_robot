SOURCES = main.c map.c path.c hw.c
OBJECTS = main.o map.o path.o hw.o

TARGET = robot

$(TARGET): $(OBJECTS)
	gcc $^ -o $@ -lwiringPi -lm

%.o: %.c
	gcc -c -Iinclude -I/usr/local/include -L/usr/local/lib $< -o $@
