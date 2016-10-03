OPENCV_INCLUDE_PATH=~/opencv-2.4.13/release/include
OPENCV_LIB_PATH=~/opencv-2.4.13/release/lib
USED_LIB=-lopencv_core -lopencv_highgui -lopencv_imgproc

all: dir bin_ucs bin_a_star bin_w_a_star demo map_gen
dir:
	mkdir bin
bin_ucs:
	g++ -std=gnu++11 -g experiment_ucs.cpp map_maker.h map_maker.cpp abstract_solver.h abstract_solver.cpp uniform_cost_search.h util.h my_heap.cpp my_hash.cpp -I$(OPENCV_INCLUDE_PATH) -L$(OPENCV_LIB_PATH) $(USED_LIB) -o bin/bin_ucs

bin_a_star:
	g++ -std=gnu++11 -g experiment_a_star.cpp map_maker.h map_maker.cpp abstract_solver.h abstract_solver.cpp a_star.h util.h my_heap.cpp my_hash.cpp -I$(OPENCV_INCLUDE_PATH) -L$(OPENCV_LIB_PATH) $(USED_LIB) -o bin/bin_a_star

bin_w_a_star:
	g++ -std=gnu++11 -g experiment_w_a_star.cpp map_maker.h map_maker.cpp abstract_solver.h abstract_solver.cpp w_a_star.h util.h my_heap.cpp my_hash.cpp -I$(OPENCV_INCLUDE_PATH) -L$(OPENCV_LIB_PATH) $(USED_LIB) -o bin/bin_w_a_star

demo:
	g++ -std=gnu++11 -g demo.cpp demo_ui.h map_maker.h map_maker.cpp abstract_solver.h abstract_solver.cpp uniform_cost_search.h a_star.h w_a_star.h util.h my_heap.cpp my_hash.cpp -I${OPENCV_INCLUDE_PATH} -L$(OPENCV_LIB_PATH) $(USED_LIB) -o bin/demo

map_gen:
	g++ -g map_gen.cpp map_maker.h map_maker.cpp -I$(OPENCV_INCLUDE_PATH) -L$(OPENCV_LIB_PATH) $(USED_LIB) -o bin/map_gen

clean:
	rm -rf bin
