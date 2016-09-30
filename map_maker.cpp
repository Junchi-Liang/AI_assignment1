#include "map_maker.h"


namespace map_maker_ns
{

	result_path::result_path()
	{
		list_row.clear();
		list_col.clear();
	}

	result_path::~result_path()
	{
		list_row.clear();
		list_col.clear();
	}

	// compute cost for the whole path given map_input as input map
	double result_path::compute_cost(const grid_map &map_input)
	{
		int i;
		double res = 0;

		for (i = 0; i < list_row.size() - 1; i++)
			res += moving_cost(list_col[i], list_row[i], list_col[i + 1], list_row[i + 1], map_input);

		return res;
	}

	// return the cost of moving from (src_col, src_row) to (dest_col, dest_row) in the map_inpu
	double result_path::moving_cost(int src_col, int src_row, int dest_col, int dest_row, const grid_map &map_input)
	{
		bool unblk_unblk = false, hard_hard = false, unblk_hard = false;
		bool horizontal_vertical = false, diag = false;
		char src_bit = map_input.read_bit(src_col, src_row), dest_bit = map_input.read_bit(dest_col, dest_row);
		bool both_in_highway = ((src_bit == 'a' || src_bit == 'b') && (dest_bit == 'a' || dest_bit == 'b'));
		double res = 1;

		if (abs(src_col - dest_col) + abs(src_row - dest_row) < 2)
			horizontal_vertical = true;
		else
			diag = true;

		unblk_unblk = ((src_bit == '1' || src_bit == 'a') && (dest_bit == '1' || dest_bit == 'a'));
		hard_hard = ((src_bit == '2' || src_bit == 'b') && (dest_bit == '2' || dest_bit == 'b'));
		unblk_hard = (!unblk_unblk && !hard_hard);

		if (horizontal_vertical && both_in_highway)
			res = COST_HIGHWAY_COEFFICIENT;

		if (unblk_unblk) // from unblocked to unblocked
		{
			if (horizontal_vertical)
				res *= COST_UNBLK_UNBLK_HV;
			else
				res *= COST_UNBLK_UNBLK_DIAG;
		} else if (hard_hard) // from hard to hard
		{
			if (horizontal_vertical)
				res *= COST_HARD_HARD_HV;
			else
				res *= COST_HARD_HARD_DIAG;
		} else
		{
			if (horizontal_vertical)
				res *= COST_UNBLK_HARD_HV;
			else
				res *= COST_UNBLK_HARD_DIAG;
		}

		return res;
	}

	// return the cell in column position_col and row position_row
	char grid_map::read_bit(int position_col, int position_row) const
	{
		return map_bit[position_col][position_row];
	}

	// set the corresponding cells in the map
	void grid_map::set_bit(int position_col, int position_row, char bit_input)
	{
		map_bit[position_col][position_row] = bit_input;
	}

	//check if the postion(position_col, position_row) is available (legal and not blocked)
	bool grid_map::check_available(int position_col, int position_row)
	{
		if (grid_map::read_bit(position_col, position_row) == BLOCKED)
			return false;
		return true;
	}

    // get the position of start cell
	int* grid_map::get_start_cell()
	{
		return start_cell;
	}
	
	// get the positiion of goal cell
	int* grid_map::get_goal_cell()
	{
		return goal_cell;
	}

	// set the position of start cell
	void grid_map::set_start_cell(int position_col, int position_row)
	{
		start_cell[0] = position_col;
		start_cell[1] = position_row;
	}

	// get the position of goal cell
	void grid_map::set_goal_cell(int position_col, int position_row)
	{
		goal_cell[0] = position_col;
		goal_cell[1] = position_row;
	}

	// get the position of the index-th hardTraverse cell
	int* grid_map::get_hardTraverse_cell(int index)
	{
		return hardTraverse_bit[index];
	}

	// set all hardTraverse cells
	void grid_map::set_hardTraverse_cell(int position_col, int position_row, int index)
	{
		hardTraverse_bit[index][0] = position_col;
		hardTraverse_bit[index][1] = position_row;
	}

	// set the size of the map
	void grid_map::set_size(int col_size, int row_size)
	{
		MAX_COLUMN = col_size;
		MAX_ROW = row_size;
	}

	// get the size of the map
	int* grid_map::get_size()
	{
		int size[2];
		size[0] = MAX_COLUMN;
		size[1] = MAX_ROW;
		return size;
	}

	grid_map::grid_map(int col, int row)
	{
		int i;
		MAX_COLUMN = col;
		MAX_ROW = row;

		map_bit = new char *[MAX_COLUMN];
		for (i = 0; i<MAX_COLUMN; i++)
			map_bit[i] = new char[MAX_ROW];
	}

	grid_map::~grid_map(){}

	// a map built by this map maker
	void map_maker::map_build() 
	{
		int i, j, k;
		// std::list<int> col_rand_highway[4], row_rand_highway[4]; // storing the cells of 4 different highways
		int col_rand_blocked, row_rand_blocked; // storing the blocked cell
		int col_dir, row_dir; // the direction of the highways
		int start_col, start_row, goal_col, goal_row; // the coordinations of start and goal cells
		srand((unsigned)time(0));
		bool highway_valid = true, highway_finished = false;

		// First step: Initialize these maps by setting all cells to unblocked cells
		for (i = 0; i < map_output.get_size()[0]; ++i)
			for (j = 0; j < map_output.get_size()[1]; ++j)
				map_output.set_bit(i, j, UNBLOCKED);

		// Then decide the placement of harder to traverse cells
		for (i = 0; i < 8; ++i) 
		{
			map_output.set_hardTraverse_cell(15 + rand() % (map_output.get_size()[0] - 30), 15 + rand() % (map_output.get_size()[1] - 30), i);

			for (j = -15; j <= 15; ++j)
			{
				for (k = -15; k <= 15; ++k)
				{
					if (rand() / double(RAND_MAX) >= 0.5)
						map_output.set_bit(map_output.get_hardTraverse_cell(i)[0] + j, map_output.get_hardTraverse_cell(i)[1] + k, HARD_TRAVERSE);
				}
			}
		}

		// Next to build four highways on the map
		for (i = 0; i < 4; ++i)
		{
			// randomly pick a new point on the edge
			if (rand() / double(RAND_MAX) <= 4.0/7)
			{
					map_output.col_rand_highway[i].push_back(1 + rand() % (map_output.get_size()[0] - 2));
					map_output.row_rand_highway[i].push_back((rand() / double(RAND_MAX) >= 0.5) ? 0 : (map_output.get_size()[1] - 1));
					col_dir = 0;
					row_dir = (map_output.row_rand_highway[i].back() == 0) ? 1 : -1;
			}
			else
			{
				map_output.col_rand_highway[i].push_back((rand() / double(RAND_MAX) >= 0.5) ? 0 : (map_output.get_size()[0] - 1));
				map_output.row_rand_highway[i].push_back(1 + rand() % (map_output.get_size()[1] - 2));
				col_dir = (map_output.col_rand_highway[i].back() == 0) ? 1 : -1;
				row_dir = 0;
			}

			// set the first highway cell in the map to be UNBLOCKED_HIGHWAY or HARD_HIGHWAY
			if (map_output.read_bit(map_output.col_rand_highway[i].back(), map_output.row_rand_highway[i].back()) == UNBLOCKED)
				map_output.set_bit(map_output.col_rand_highway[i].back(), map_output.row_rand_highway[i].back(), UNBLOCKED_HIGHWAY);
			else if (map_output.read_bit(map_output.col_rand_highway[i].back(), map_output.row_rand_highway[i].back()) == HARD_TRAVERSE)
				map_output.set_bit(map_output.col_rand_highway[i].back(), map_output.row_rand_highway[i].back(), HARD_HIGHWAY);
			else
			{
				i -= 1;
				map_output.col_rand_highway[i].clear();
				map_output.row_rand_highway[i].clear();
				break;
			}

			// break until the highway hit the edge or errors
			while (1)
			{
				
				for (j = 0; j < 20; ++j)
				{
					// continue and push the next cell into highway
					map_output.col_rand_highway[i].push_back(map_output.col_rand_highway[i].back() + col_dir);
					map_output.row_rand_highway[i].push_back(map_output.row_rand_highway[i].back() + row_dir);

					// set the cell in the map to be UNBLOCKED_HIGHWAY or HARD_HIGHWAY
					if (map_output.read_bit(map_output.col_rand_highway[i].back(), map_output.row_rand_highway[i].back()) == UNBLOCKED)
						map_output.set_bit(map_output.col_rand_highway[i].back(), map_output.row_rand_highway[i].back(), UNBLOCKED_HIGHWAY);
					else if (map_output.read_bit(map_output.col_rand_highway[i].back(), map_output.row_rand_highway[i].back()) == HARD_TRAVERSE)
						map_output.set_bit(map_output.col_rand_highway[i].back(), map_output.row_rand_highway[i].back(), HARD_HIGHWAY);
					else
					{
						map_output.col_rand_highway[i].pop_back();
						map_output.row_rand_highway[i].pop_back();
						highway_valid = false;
						break;
					}

					// check if this cell is edge or not
					if (map_output.col_rand_highway[i].back() == 0 || map_output.row_rand_highway[i].back() == 0 || \
						map_output.col_rand_highway[i].back() == map_output.get_size()[0] - 1 || map_output.row_rand_highway[i].back() == map_output.get_size()[1] - 1)
					{
						if (map_output.col_rand_highway[i].size() < 100)
						{
							highway_valid = false;
							break;
						}
						else
						{
							highway_finished = true;
							break;
						}
					}
				}

				// if highway_valid is false we need to delete all the current cells in highway
				// if highway_valid is true and highway_finished is false we need to take turns
				// if highway_valid is true and highway_finished is true we need to set the highway to the map							
				if (highway_valid == false)
				{
					while (map_output.col_rand_highway[i].size() != 0)
					{
						if (map_output.read_bit(map_output.col_rand_highway[i].back(), map_output.row_rand_highway[i].back()) == UNBLOCKED_HIGHWAY)
							map_output.set_bit(map_output.col_rand_highway[i].back(), map_output.row_rand_highway[i].back(), UNBLOCKED);
						else if (map_output.read_bit(map_output.col_rand_highway[i].back(), map_output.row_rand_highway[i].back()) == HARD_HIGHWAY)
							map_output.set_bit(map_output.col_rand_highway[i].back(), map_output.row_rand_highway[i].back(), HARD_TRAVERSE);
						map_output.col_rand_highway[i].pop_back();
						map_output.row_rand_highway[i].pop_back();
					}
					highway_valid = true;
					highway_finished = false;
					map_output.col_rand_highway[i].clear();
					map_output.row_rand_highway[i].clear();
					i -= 1;
					break;
				} 
				else
				{
					if (highway_finished == false)
					{
						if (rand() / double(RAND_MAX) >= 0.6) // go straight means no direction changes
						{
							for (j = 1; j < 8; j += 2)
								if (col_dir == DELTA_COL[j] && row_dir == DELTA_ROW[j])
									break;
							if (rand() / double(RAND_MAX)<= 0.5) // turn left
							{								
								if (j == 7)
									j = -1;
								col_dir = DELTA_COL[j + 2];
								row_dir = DELTA_ROW[j + 2];
							}
							else // turn right
							{
								if (j == 1)
									j = 9;
								col_dir = DELTA_COL[j - 2];
								row_dir = DELTA_ROW[j - 2];
							}
						}
					}
					else
					{
						break;
					}
				}
			}
			
			highway_valid = true;
			highway_finished = false;
		}
		
		// select the blocked cells (20%)
		for (i = 0; i < 3840; ++i)
		{	
			col_rand_blocked = rand() % 160;
			row_rand_blocked = rand() % 120;
			if ((map_output.read_bit(col_rand_blocked, row_rand_blocked) != HARD_HIGHWAY) && \
				(map_output.read_bit(col_rand_blocked, row_rand_blocked) != UNBLOCKED_HIGHWAY) && \
				(map_output.read_bit(col_rand_blocked, row_rand_blocked) != BLOCKED))
				map_output.set_bit(col_rand_blocked, row_rand_blocked, BLOCKED);
			else
				--i;
		}

		// Decide the start cell and the goal cell
		// start cell
		do
		{
			start_col = rand() % 160;
			start_row = rand() % 120;
		} while (start_col>20 && start_col<140 && start_row>20 && start_row<100);
		// goal cell
		do
		{
			goal_col = rand() % 160;
			goal_row = rand() % 120;
		} while ((goal_col>20 && goal_col<140 && goal_row>20 && goal_row<100) \
			|| abs(goal_row-start_row)+abs(goal_col-start_col)<100);
		map_output.set_start_cell(start_col, start_row);
		map_output.set_goal_cell(goal_col, goal_row);
	}

	// store the map_output into a text file in the path given by dest
	void map_maker::write_text_to_disk(char* dest)
	{
		std::ofstream fout;
		int i, j;

		fout.open(dest);
		fout << map_output.get_start_cell()[0] << "," << map_output.get_start_cell()[1] << std::endl;
		fout << map_output.get_goal_cell()[0] << "," << map_output.get_goal_cell()[1] << std::endl;

		for (i = 0; i < 8; ++i)
			fout << map_output.get_hardTraverse_cell(i)[0] << "," << map_output.get_hardTraverse_cell(i)[1] << std::endl;

		fout << map_output.get_size()[1] << "," << map_output.get_size()[0] << std::endl;

		// build the highway map
		char** highway_map;
		highway_map = new char *[map_output.get_size()[0]];
		for (i = 0; i<map_output.get_size()[0]; i++)
			highway_map[i] = new char[map_output.get_size()[1]];

		for (i = 0; i < map_output.get_size()[0]; ++i)
			for (j = 0; j < map_output.get_size()[1]; ++j)
				highway_map[i][j] = '0';

		std::list<int> highway[2];
		
		for (i = 0; i < 4; ++i) {
			highway[0].clear();
			highway[1].clear();
			std::copy(map_output.col_rand_highway[i].begin(), map_output.col_rand_highway[i].end(), std::back_inserter(highway[0]));
			std::copy(map_output.row_rand_highway[i].begin(), map_output.row_rand_highway[i].end(), std::back_inserter(highway[1]));
			while (highway[0].size() != 0)
			{
				highway_map[highway[0].back()][highway[1].back()] = '1' + i;
				highway[0].pop_back();
				highway[1].pop_back();
			}
		}


		for (i = map_output.get_size()[1] - 1; i >= 0; --i)
		{
			for (j = 0; j < map_output.get_size()[0] - 1; ++j)
			{
				if (highway_map[j][i] == '0')
					fout << map_output.read_bit(j, i) << ",";
				else
					fout << map_output.read_bit(j, i) << highway_map[j][i] << ",";
			}
				
			if (highway_map[j][i] == '0')
				fout << map_output.read_bit(j, i) << "\n";
			else
				fout << map_output.read_bit(j, i) << highway_map[j][i] << "\n";
		}

		fout.close();
	}

	// display the map_input as an image
	cv::Mat map_maker::show_map_img()
	{
		int amplify_num = 5;
		cv::Mat map_image(map_output.get_size()[1] *amplify_num, map_output.get_size()[0] *amplify_num\
			,CV_8UC3, cv::Scalar(COLOR_BLOCKED[0], COLOR_BLOCKED[1], COLOR_BLOCKED[2]));
		int i, j, k, h;

		// color all kinds of cells
		for (i = 0; i < map_output.get_size()[1]; ++i)
		{
			for (j = 0; j < map_output.get_size()[0]; ++j)
			{
				if (map_output.read_bit(j, 119 - i) == UNBLOCKED) // set the color of unblocked cells to white
				{
					for (k = 0; k < amplify_num; ++k)
					{
						for (h = 0; h < amplify_num; ++h)
						{
							map_image.at<cv::Vec3b>(i*amplify_num + k, j*amplify_num + h)[0] = COLOR_UNBLOCKED[2];
							map_image.at<cv::Vec3b>(i*amplify_num + k, j*amplify_num + h)[1] = COLOR_UNBLOCKED[1];
							map_image.at<cv::Vec3b>(i*amplify_num + k, j*amplify_num + h)[2] = COLOR_UNBLOCKED[0];
						}
					}
				}
				else if (map_output.read_bit(j, 119 - i) == HARD_TRAVERSE) // set the color of hardTraverse cells to grey
				{
					for (k = 0; k < amplify_num; ++k)
					{
						for (h = 0; h < amplify_num; ++h)
						{
							map_image.at<cv::Vec3b>(i*amplify_num + k, j*amplify_num + h)[0] = COLOR_HARD_TRAVERSE[2];
							map_image.at<cv::Vec3b>(i*amplify_num + k, j*amplify_num + h)[1] = COLOR_HARD_TRAVERSE[1];
							map_image.at<cv::Vec3b>(i*amplify_num + k, j*amplify_num + h)[2] = COLOR_HARD_TRAVERSE[0];
						}
					}
				}
				else if (map_output.read_bit(j, 119 - i) == HARD_HIGHWAY) // set the color of hardTraverse highways cells to sky blue
				{
					for (k = 0; k < amplify_num; ++k)
					{
						for (h = 0; h < amplify_num; ++h)
						{
							map_image.at<cv::Vec3b>(i*amplify_num + k, j*amplify_num + h)[0] = COLOR_HARD_TRAVERSE_HIGHWAY[2];
							map_image.at<cv::Vec3b>(i*amplify_num + k, j*amplify_num + h)[1] = COLOR_HARD_TRAVERSE_HIGHWAY[1];
							map_image.at<cv::Vec3b>(i*amplify_num + k, j*amplify_num + h)[2] = COLOR_HARD_TRAVERSE_HIGHWAY[0];
						}
					}
				}
				else if (map_output.read_bit(j, 119 - i) == UNBLOCKED_HIGHWAY) // set the color of unblocked highways cells to blue
				{
					for (k = 0; k < amplify_num; ++k)
					{
						for (h = 0; h < amplify_num; ++h)
						{
							map_image.at<cv::Vec3b>(i*amplify_num + k, j*amplify_num + h)[0] = COLOR_UNBLOCKED_HIGHWAY[2];
							map_image.at<cv::Vec3b>(i*amplify_num + k, j*amplify_num + h)[1] = COLOR_UNBLOCKED_HIGHWAY[1];
							map_image.at<cv::Vec3b>(i*amplify_num + k, j*amplify_num + h)[2] = COLOR_UNBLOCKED_HIGHWAY[0];
						}
					}
				}
			}
		}

		// color the start and goal cell
		for (k = 0; k < amplify_num; ++k)
		{
			for (h = 0; h < amplify_num; ++h)
			{
				map_image.at<cv::Vec3b>((119 - map_output.get_start_cell()[1]) * amplify_num + k, map_output.get_start_cell()[0]*amplify_num + h)[0] = COLOR_START[2];
				map_image.at<cv::Vec3b>((119 - map_output.get_start_cell()[1]) * amplify_num + k, map_output.get_start_cell()[0]*amplify_num + h)[1] = COLOR_START[1];
				map_image.at<cv::Vec3b>((119 - map_output.get_start_cell()[1]) * amplify_num + k, map_output.get_start_cell()[0]*amplify_num + h)[2] = COLOR_START[0];
			}
		}
		for (k = 0; k < amplify_num; ++k)
		{
			for (h = 0; h < amplify_num; ++h)
			{
				map_image.at<cv::Vec3b>((119 - map_output.get_goal_cell()[1]) * amplify_num + k, map_output.get_goal_cell()[0] * amplify_num + h)[0] = COLOR_GOAL[2];
				map_image.at<cv::Vec3b>((119 - map_output.get_goal_cell()[1]) * amplify_num + k, map_output.get_goal_cell()[0] * amplify_num + h)[1] = COLOR_GOAL[1];
				map_image.at<cv::Vec3b>((119 - map_output.get_goal_cell()[1]) * amplify_num + k, map_output.get_goal_cell()[0] * amplify_num + h)[2] = COLOR_GOAL[0];
			}
		}
		

		return map_image;
	}

	// store the map_output into an image file in the path given by dest
	void map_maker::write_img_to_disk(char* dest)
	{
		cv::Mat mat_img = show_map_img();
		cv::imwrite(dest, mat_img);
	}

	// read a text file in path given by src and store the map into map_loaded
	void map_maker::read_text(char* src)
	{
		std::ifstream fin;
		fin.open(src);
		char comma;
		int i, j;
		int col_start, row_start;
		int col_goal, row_goal;
		int hardTraverse[2];
		int col_num, row_num;
		char bit_type;
		int highway_num;

		// clear col_rand_highway & row_rand_highway
		for (i = 0; i < 4; ++i)
		{
			map_output.col_rand_highway[i].clear();
			map_output.row_rand_highway[i].clear();
		}

		fin >> col_start >> comma >> row_start;
		map_output.set_start_cell(col_start, row_start);

		fin >> col_goal >> comma >> row_goal;
		map_output.set_goal_cell(col_goal, row_goal);

		for (i = 0; i < 8; ++i)
		{
			fin >> hardTraverse[0] >> comma >> hardTraverse[1];
			map_output.set_hardTraverse_cell(hardTraverse[0], hardTraverse[1], i);
		}

		fin >> row_num >> comma >> col_num;
		map_output.set_size(col_num, row_num);

		for (i = map_output.get_size()[1] - 1; i >= 0; --i) {
			for (j = 0; j < map_output.get_size()[0]; ++j)
			{
				fin >> bit_type;
				if (bit_type == ',')
					j -= 1;
				else
				{
					map_output.set_bit(j, i, bit_type);
					if (bit_type == 'a' || bit_type == 'b')
					{
						fin >> highway_num;
						map_output.col_rand_highway[highway_num-1].push_back(j);
						map_output.row_rand_highway[highway_num-1].push_back(i);
					}

				}
				
			}
		}
	}
}

