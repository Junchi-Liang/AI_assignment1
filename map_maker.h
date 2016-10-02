#ifndef MAP_MAKER_H
#define MAP_MAKER_H

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <iterator>
#include <list>
#include <time.h>
#include <fstream>
#include <opencv/highgui.h>
#include <algorithm>

namespace map_maker_ns
{

	const char BLOCKED = '0', UNBLOCKED = '1', HARD_TRAVERSE = '2', UNBLOCKED_HIGHWAY = 'a', HARD_HIGHWAY = 'b';

	// directions
	const int MAX_DIR = 8;										// 6 5 4
	const int DELTA_COL[MAX_DIR] = {-1, 0, 1, 1, 1, 0, -1, -1}; // 7   3
	const int DELTA_ROW[MAX_DIR] = {-1, -1, -1, 0, 1, 1, 1, 0}; // 0 1 2

	// cost of diff routes
	const double COST_UNBLK_UNBLK_HV = 1;
	const double COST_UNBLK_UNBLK_DIAG = sqrt(2);
	const double COST_HARD_HARD_HV = 2;
	const double COST_HARD_HARD_DIAG = sqrt(8);
	const double COST_UNBLK_HARD_HV = 1.5;
	const double COST_UNBLK_HARD_DIAG = (sqrt(2) + sqrt(8)) / 2;
	const double COST_HIGHWAY_COEFFICIENT = 1 / 4;

	// color of different kinds of cell
	const int COLOR_BLOCKED[3] = { 0,0,0 }; // Black
	const int COLOR_UNBLOCKED[3] = { 255,255,255 }; // White
	const int COLOR_HARD_TRAVERSE[3] = { 192,192,192 }; // Grey
	const int COLOR_UNBLOCKED_HIGHWAY[3] = { 0,0,255 }; // Blue
	const int COLOR_HARD_TRAVERSE_HIGHWAY[3] = { 135,206,235 }; // Sky Blue
	const int COLOR_START[3] = { 0,255,0 }; // Green
	const int COLOR_GOAL[3] = { 255,0,0 }; // Red
	const int COLOR_RESULT[3] = { 255,255,0 };


	// map data, assume both columns and rows start from 0
	class grid_map
	{
		private:
			int MAX_COLUMN, MAX_ROW;
			char** map_bit;
			int hardTraverse_bit[8][2]; // storing the centers of 8 different harder traveral area
			int start_cell[2];
			int goal_cell[2];
		public:
			std::list<int> col_rand_highway[4], row_rand_highway[4]; // storing the cells of 4 different highways
			grid_map(int MAX_COLUMN=160,int MAX_ROW=120);
			~grid_map();
			void set_size(int MAX_COLUMN, int MAX_ROW);
			int get_col_size() const;
			int get_row_size() const;
			char read_bit(int position_col, int position_row) const; // return the cell in column position_col and row position_row
			void set_bit(int position_col, int position_row, char bit_input); // set the cell in column position_col and row position_row as bit_input 
			bool check_available(int position_col, int position_row) const;  // check if the postion(position_col, position_row) is available (legal and not blocked)
			int get_col_start() const; // get the position of start cell
			int get_row_start() const;
			int get_col_goal() const; // get the positiion of goal cell
			int get_row_goal() const; 
			void set_start_cell(int position_col, int position_row); // set the position of start cell
			void set_goal_cell(int position_col, int position_row); // set the position of goal cell
			int get_col_hardTraverse(int index) const; // get the position of the index-th hardTraverse cell
			int get_row_hardTraverse(int index) const;
			void set_hardTraverse_cell(int position_col, int position_row, int index); // set all hardTraverse cells
	};

	// data structure for the result path
	class result_path
	{
		public:
			result_path();
			~result_path();
			std::vector<int> list_row, list_col;
			double compute_cost(const grid_map &map_input); // compute cost for the whole path given map_input as input map
			static double moving_cost(int src_col, int src_row, int dest_col, int dest_row, const grid_map &map_input); // return the cost of moving from (src_col, src_row) to (dest_col, dest_row) in the map_input
	};

	// map maker
	class map_maker
 	{
		public:
			grid_map map_output; // a map built by this map maker
//		public:
			void map_build(); // build a new map and store it into map_output
			void write_text_to_disk(char* dest); // store the map_output into a text file in the path given by dest
			void write_img_to_disk(char* dest); // store the map_output into an image file in the path given by dest
			static cv::Mat show_map_img(const grid_map &map_input); // display and return the map_input as an image
			cv::Mat show_map_img(); // display and return the map_input as an image
			static cv::Mat show_result(const grid_map &map_input, const result_path &result); // optional: display the map_input and the result
			static void read_text(char* src, grid_map &map_loaded); // read a text file in path given by src and store the map into map_loaded
			void read_text(char* src); // read a text file in path given
			void set_map_start_goal(); // change the start and goal cell in the map
			
	};

}


#endif
