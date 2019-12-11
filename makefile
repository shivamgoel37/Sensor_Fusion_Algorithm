CC=gcc
CFLAGS=-std=c++17

INCLUDEGSL=-I lib
LINKGSL=-L lib/linker

build_folder := $(shell mkdir -p build)
bin_folder := $(shell mkdir -p bin)

all: 
	$(CC) -Wall $(INCLUDEGSL) -c src/program_final.c -o build/program_final.o
	$(CC) -g -o bin/FINAL $(LINKGSL) build/program_final.o -lgsl -lgslcblas -lm
clean:
	rm -f FINAL build/*.o bin/*

	