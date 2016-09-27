#include "map_maker.h"

int main()
{
	map_maker_ns::map_maker old_map;
	old_map.read_text("data.txt");
	old_map.show_map_img();
	cv::Mat old_map_img = old_map.show_map_img();
	old_map.write_img_to_disk("old_map.jpg");

	map_maker_ns::map_maker new_map;	
	new_map.map_build();
	new_map.write_text_to_disk("new_data.txt");
	cv::Mat new_map_img = new_map.show_map_img();
	new_map.write_img_to_disk("new_map.jpg");

	cv::namedWindow("Map_CS520_old");
	cv::imshow("Map_CS520_old", old_map_img);
	cv::namedWindow("Map_CS520_new");
	cv::imshow("Map_CS520_new", new_map_img);
	cv::waitKey();
	return 0;
}

