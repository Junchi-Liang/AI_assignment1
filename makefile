OPENCV_INCLUDE_PATH=~/opencv-2.4.13/release/include
OPENCV_LIB_PATH=~/opencv-2.4.13/release/lib
USED_LIB=-lopencv_core -lopencv_highgui -lopencv_imgproc

all: dir test_bin bin_ucs bin_a_star bin_w_a_star map_gen
dir:
	mkdir bin
test_bin:
	g++ -std=gnu++11 -g first_test.cpp map_maker.h map_maker.cpp abstract_solver.h abstract_solver.cpp uniform_cost_search.h util.h my_heap.cpp my_hash.cpp -I$(OPENCV_INCLUDE_PATH) -L$(OPENCV_LIB_PATH) $(USED_LIB) -o bin/test_bin

bin_ucs:
	g++ -std=gnu++11 -g experiment_ucs.cpp map_maker.h map_maker.cpp abstract_solver.h abstract_solver.cpp uniform_cost_search.h util.h my_heap.cpp my_hash.cpp -I$(OPENCV_INCLUDE_PATH) -L$(OPENCV_LIB_PATH) $(USED_LIB) -o bin/bin_ucs

bin_a_star:
	g++ -std=gnu++11 -g experiment_a_star.cpp map_maker.h map_maker.cpp abstract_solver.h abstract_solver.cpp a_star.h util.h my_heap.cpp my_hash.cpp -I$(OPENCV_INCLUDE_PATH) -L$(OPENCV_LIB_PATH) $(USED_LIB) -o bin/bin_a_star

bin_w_a_star:
	g++ -std=gnu++11 -g experiment_w_a_star.cpp map_maker.h map_maker.cpp abstract_solver.h abstract_solver.cpp w_a_star.h util.h my_heap.cpp my_hash.cpp -I$(OPENCV_INCLUDE_PATH) -L$(OPENCV_LIB_PATH) $(USED_LIB) -o bin/bin_w_a_star

map_gen:
	g++ -g map_gen.cpp map_maker.h map_maker.cpp -I$(OPENCV_INCLUDE_PATH) -L$(OPENCV_LIB_PATH) $(USED_LIB) -o bin/map_gen

clean:
	rm -rf bin
