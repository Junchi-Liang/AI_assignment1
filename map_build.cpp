#include "map_maker.h"
#include "abstract_heuristic.h"
#include "uniform_cost_search.h"

int main()
{

	// read olddata to create a new map
	map_maker_ns::map_maker old_map;
	old_map.read_text("map2.txt");
	cv::Mat old_map_img = old_map.show_map_img();
	cv::namedWindow("Map_CS520_old");
	cv::imshow("Map_CS520_old", old_map_img);
	old_map.write_img_to_disk("old_map.jpg");
	cvSetMouseCallback("Map_CS520_old", old_map.mouse_event, NULL);
	cv::waitKey();

/*
	for (int i = 0; i < 5; ++i) {
		map_maker_ns::map_maker map;
		map.map_build();
		for (int j = 0; j < 10; ++j) {
			map.reset_map_start_goal(map.map_output);
			char file_name[10];
			sprintf(file_name, "map/%d%d", i, j);
			map.write_text_to_disk(file_name);
		}
	}
*/
	
/*
	cv::namedWindow("cell_information");
    // create a random new map and show the result
	char* window_name = "map_cs520_new";
	map_maker_ns::map_maker new_map;	
	new_map.map_build();
	new_map.write_text_to_disk("new_data.txt");
	cv::Mat new_map_img = new_map.show_map_img();

	heuristic_ns::abstract_heuristic *heuristic = new heuristic_ns::abstract_heuristic();
	solver_ns::uniform_cost_search solver(heuristic);
	
	map_maker_ns::result_path result;
	int expanded;
	bool temp = solver.solve(new_map.map_output.get_col_start(), new_map.map_output.get_row_start(), new_map.map_output.get_col_goal(), \
		new_map.map_output.get_row_goal(), new_map.map_output, result, expanded);
	solver.store_table("f_g_h.txt");
	delete heuristic;
	cv::Mat new_map_result = new_map.show_result(new_map.map_output, result);

	cv::namedWindow(window_name);
	cvSetMouseCallback(window_name, new_map.mouse_event, NULL);
	cv::imshow(window_name, new_map_result);
	new_map.write_img_to_disk("new_map.jpg");

	cv::waitKey();
*/
	return 0;
}

