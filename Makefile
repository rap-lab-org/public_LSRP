build_dir="build"
flag="Debug"

all: build
fast dev: all

dev: flag = Debug
fast: flag = Release

.PHONY: gen build clean
gen:
	@mkdir -p ${build_dir}
	@echo "cmake -B${build_dir} -H. -DCMAKE_BUILD_TYPE=${flag} -DCMAKE_EXPORT_COMPILE_COMMANDS=1"
	@eval "cmake -B${build_dir} -H. -DCMAKE_BUILD_TYPE=${flag} -DCMAKE_EXPORT_COMPILE_COMMANDS=1"

build: gen
	@echo "cd ${build_dir} && make -j"
	@eval "cd ${build_dir} && make -j"

clean:
	@echo "cd ${build_dir} && make clean"
	@eval "cd ${build_dir} && make clean"
