#ifndef MAP_MAKER_H
#define MAP_MAKER_H

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>

namespace map_maker_ns
{

	const int MAX_COLUMN = 160, MAX_ROW = 120;

	const char BLOCKED = '0', UNBLOCKED = '1', HARD_TRAVERSE = '2', UNBLOCKED_HIGHWAY = 'a', HARD_HIGHWAY = 'b';

	// directions
	const int MAX_DIR = 8;
	const int DELTA_COL[MAX_DIR] = {-1, -1, -1, 0, 0, 1, 1, 1};
	const int DELTA_ROW[MAX_DIR] = {-1, 0, 1, -1, 1, -1, 0, 1};

	// cost
	const double COST_UNBLK_UNBLK_HV = 1;
	const double COST_UNBLK_UNBLK_DIAG = sqrt(2);
	const double COST_HARD_HARD_HV = 2;
	const double COST_HARD_HARD_DIAG = sqrt(8);
	const double COST_UNBLK_HARD_HV = 1.5;
	const double COST_UNBLK_HARD_DIAG = (sqrt(2) + sqrt(8)) / 2;
	const double COST_HIGHWAY_COEFFICIENT = 1 / 4;

	// map data
	class grid_map
	{
		private:
			char map_bit[MAX_COLUMN][MAX_ROW];
		public:
			char read_bit(int position_col, int position_row) const; // return the cell in column position_col and row position_row
			void set_bit(int position_col, int position_row, char bit_input); // set the cell in column position_col and row position_row as bit_input 
			static bool check_available(int position_col, int position_row);  // check if the postion(position_col, position_row) is available (legal and not blocked)
	};

	// data structure for the result path
	class result_path
	{
		result_path();
		~result_path();
		public:
			std::vector<int> list_row, list_col;
			double compute_cost(const grid_map &map_input); // compute cost for the whole path given map_input as input map
			static double moving_cost(int src_col, int src_row, int dest_col, int dest_row, const grid_map &map_input); // return the cost of moving from (src_col, src_row) to (dest_col, dest_row) in the map_input
	};

	// map maker
	class map_maker
 	{
		private:
			grid_map map_output; // a map built by this map maker
		public:
			void map_build(); // build a new map and store it into map_output
			void write_text_to_disk(char* dest); // store the map_output into a text file in the path given by dest
			void write_img_to_disk(char* dest); // store the map_output into an image file in the path given by dest
			static void show_map_img(const grid_map &map_input); // display the map_input as an image
			static void show_result(const grid_map &map_input, const result_path &result); // optional: display the map_input and the result
			static void read_text(char* src, grid_map &map_loaded); // read a text file in path given by src and store the map into map_loaded
	};

}


#endif
