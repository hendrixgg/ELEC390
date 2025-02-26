# Jacob Chisholm

.PHONY: all build cmake clean format

BUILD_DIR := build
BUILD_TYPE ?= Debug

PNAME := PIX_driver

all: build

${BUILD_DIR}/Makefile:
	cmake \
		-DCMAKE_C_COMPILER=gcc\
		-DCMAKE_CXX_COMPILER=g++\
		-B${BUILD_DIR} \
		-DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		-G "Unix Makefiles" \

cmake: ${BUILD_DIR}/Makefile

build: cmake
	$(MAKE) -C ${BUILD_DIR} --no-print-directory

SRCS := $(shell find . -name '*.[ch]' -or -name '*.[ch]pp')
%.format: %
	clang-format -i $<
format: $(addsuffix .format, ${SRCS})

clean:
	rm -rf $(BUILD_DIR)

run: build
	./${BUILD_DIR}/${PNAME}


valgrind: build
	valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --error-limit=no --log-file=valgrindout.txt ./build/${PNAME}
