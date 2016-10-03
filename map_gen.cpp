#include "map_maker.h"


int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("usage: map_gen text_stored_path visualize(optional, y or n) image_stored_path(optional)\n");
		return 0;
	}

	map_maker_ns::map_maker new_map;
	new_map.map_build();
	new_map.write_text_to_disk((char*)(argv[1]));

	if (argc > 2 && strcmp((char*)(argv[2]), "y") == 0)
	{
		cv::Mat new_map_img = new_map.show_map_img();
		cv::namedWindow("map");
		cv::imshow("map", new_map_img);
		cv::waitKey();
	}

	if (argc > 3)
		new_map.write_img_to_disk((char*)(argv[3]));

	return 0;
}

