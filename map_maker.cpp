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
	bool grid_map::check_available(int position_col, int position_row) const
	{
		if (position_col < 0 || position_col >= get_col_size() || position_row < 0 || position_row >= get_row_size())
			return false;
		if (read_bit(position_col, position_row) == BLOCKED)
			return false;
		return true;
	}

    // get the col position of start cell
	int grid_map::get_col_start() const
	{
		return start_cell[0];
	}

	// get the row position of start cell
	int grid_map::get_row_start() const
	{
		return start_cell[1];
	}
	
	// get the col positiion of goal cell
	int grid_map::get_col_goal() const
	{
		return goal_cell[0];
	}

	// get the row position of goal cell
	int grid_map::get_row_goal() const
	{
		return goal_cell[1];
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

	// get the col position of the index-th hardTraverse cell
	int grid_map::get_col_hardTraverse(int index) const
	{
		return hardTraverse_bit[index][0];
	}

	// get the row position of the index-th hardTraverse cell
	int grid_map::get_row_hardTraverse(int index) const
	{
		return hardTraverse_bit[index][1];
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

	// get the Max col of the map
	int grid_map::get_col_size() const
	{
		return MAX_COLUMN;
	}

	// get the Max row of the map
	int grid_map::get_row_size() const
	{
		return MAX_ROW;
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
		for (i = 0; i < map_output.get_col_size(); ++i)
			for (j = 0; j < map_output.get_row_size(); ++j)
				map_output.set_bit(i, j, UNBLOCKED);

		// Then decide the placement of harder to traverse cells
		for (i = 0; i < 8; ++i) 
		{
			map_output.set_hardTraverse_cell(15 + rand() % (map_output.get_col_size() - 30), 15 + rand() % (map_output.get_row_size() - 30), i);

			for (j = -15; j <= 15; ++j)
			{
				for (k = -15; k <= 15; ++k)
				{
					if (rand() / double(RAND_MAX) >= 0.5)
						map_output.set_bit(map_output.get_col_hardTraverse(i) + j, map_output.get_row_hardTraverse(i) + k, HARD_TRAVERSE);
				}
			}
		}

		// Next to build four highways on the map
		for (i = 0; i < 4; ++i)
		{
			// randomly pick a new point on the edge
			if (rand() / double(RAND_MAX) <= 4.0/7)
			{
					map_output.col_rand_highway[i].push_back(1 + rand() % (map_output.get_col_size() - 2));
					map_output.row_rand_highway[i].push_back((rand() / double(RAND_MAX) >= 0.5) ? 0 : (map_output.get_row_size() - 1));
					col_dir = 0;
					row_dir = (map_output.row_rand_highway[i].back() == 0) ? 1 : -1;
			}
			else
			{
				map_output.col_rand_highway[i].push_back((rand() / double(RAND_MAX) >= 0.5) ? 0 : (map_output.get_col_size() - 1));
				map_output.row_rand_highway[i].push_back(1 + rand() % (map_output.get_row_size() - 2));
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
				map_output.col_rand_highway[i].clear();
				map_output.row_rand_highway[i].clear();
				i -= 1;
				continue;
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
						map_output.col_rand_highway[i].back() == (map_output.get_col_size() - 1) || map_output.row_rand_highway[i].back() == (map_output.get_row_size() - 1))
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
		for (i = 0; i < map_output.get_col_size()*map_output.get_row_size()*0.2; ++i)
		{	
			col_rand_blocked = rand() % map_output.get_col_size();
			row_rand_blocked = rand() % map_output.get_row_size();
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
			start_col = 1 + rand() % (map_output.get_col_size() - 2);
			start_row = 1 + rand() % (map_output.get_row_size() - 2);
		} while ((start_col>20 && start_col<(map_output.get_col_size()-20) && start_row>20 && start_row<(map_output.get_row_size()-20)) || !map_output.check_available(start_col,start_row));
		// goal cell
		do
		{
			goal_col = 1 + rand() % (map_output.get_col_size() - 2);
			goal_row = 1 + rand() % (map_output.get_row_size() - 2);
		} while ((goal_col>20 && goal_col<(map_output.get_col_size() - 20) && goal_row>20 && goal_row<(map_output.get_row_size() - 20)) || \
			!map_output.check_available(goal_col, goal_row) || abs(goal_row-start_row)+abs(goal_col-start_col)<100);
		map_output.set_start_cell(start_col, start_row);
		map_output.set_goal_cell(goal_col, goal_row);
	}

	// store the map_output into a text file in the path given by dest
	void map_maker::write_text_to_disk(char* dest)
	{
		std::ofstream fout;
		int i, j;

		fout.open(dest);
		fout << map_output.get_row_start() << "," << map_output.get_col_start() << std::endl;
		fout << map_output.get_row_goal() << "," << map_output.get_col_goal() << std::endl;

		for (i = 0; i < 8; ++i)
			fout << map_output.get_row_hardTraverse(i) << "," << map_output.get_col_hardTraverse(i) << std::endl;

		fout << map_output.get_row_size() << "," << map_output.get_col_size() << std::endl;

		// build the highway map
		char** highway_map;
		highway_map = new char *[map_output.get_col_size()];
		for (i = 0; i<map_output.get_col_size(); i++)
			highway_map[i] = new char[map_output.get_row_size()];

		for (i = 0; i < map_output.get_col_size(); ++i)
			for (j = 0; j < map_output.get_row_size(); ++j)
				highway_map[i][j] = 'n';

		std::list<int> highway[2];
		
		for (i = 0; i < 4; ++i) {
			highway[0].clear();
			highway[1].clear();
			std::copy(map_output.col_rand_highway[i].begin(), map_output.col_rand_highway[i].end(), std::back_inserter(highway[0]));
			std::copy(map_output.row_rand_highway[i].begin(), map_output.row_rand_highway[i].end(), std::back_inserter(highway[1]));
			while (highway[0].size() != 0)
			{
				highway_map[highway[0].back()][highway[1].back()] = '0' + i;
				highway[0].pop_back();
				highway[1].pop_back();
			}
		}


		for (i = map_output.get_row_size() - 1; i >= 0; --i)
		{
			for (j = 0; j < map_output.get_col_size() - 1; ++j)
			{
				if (highway_map[j][i] == 'n')
					fout << map_output.read_bit(j, i) << ",";
				else
					fout << map_output.read_bit(j, i) << highway_map[j][i] << ",";
			}
				
			if (highway_map[j][i] == 'n')
				fout << map_output.read_bit(j, i) << "\n";
			else
				fout << map_output.read_bit(j, i) << highway_map[j][i] << "\n";
		}

		fout.close();
	}

	// optional: display the map_input and the result
	cv::Mat map_maker::show_result(const grid_map &map_input, const result_path &result)
	{
		cv::Mat result_map;
		result_map = show_map_img(map_input);		
		
		if (result.list_col.size() == 0)
		{
			printf("NOT FOUND!");
			return result_map;
		}

		int i;
		int k, h;
		int amplify_num = 5;

		for (i = 1; i < result.list_row.size()-1; ++i)
		{
			for (k = 1; k < amplify_num-1; ++k)
			{
				for (h = 1; h < amplify_num-1; ++h)
				{
					result_map.at<cv::Vec3b>((map_input.get_row_size() - 1 - result.list_row[i])*amplify_num + k, result.list_col[i] * amplify_num + h)[0] = COLOR_RESULT[2];
					result_map.at<cv::Vec3b>((map_input.get_row_size() - 1 - result.list_row[i])*amplify_num + k, result.list_col[i] * amplify_num + h)[1] = COLOR_RESULT[1];
					result_map.at<cv::Vec3b>((map_input.get_row_size() - 1 - result.list_row[i])*amplify_num + k, result.list_col[i] * amplify_num + h)[2] = COLOR_RESULT[0];
				}
			}
		}

		return result_map;
	}

	// display the map_input as an image using an exist grid_map
	cv::Mat map_maker::show_map_img(const grid_map &map_input)
	{
		int amplify_num = 5;
		cv::Mat map_image(map_input.get_row_size() * amplify_num, map_input.get_col_size() * amplify_num\
			,CV_8UC3, cv::Scalar(COLOR_BLOCKED[0], COLOR_BLOCKED[1], COLOR_BLOCKED[2]));
		int i, j, k, h;

		// color all kinds of cells
		for (i = 0; i < map_input.get_row_size(); ++i)
		{
			for (j = 0; j < map_input.get_col_size(); ++j)
			{
				if (map_input.read_bit(j, map_input.get_row_size()-1 - i) == UNBLOCKED) // set the color of unblocked cells to white
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
				else if (map_input.read_bit(j, map_input.get_row_size() - 1 - i) == HARD_TRAVERSE) // set the color of hardTraverse cells to grey
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
				else if (map_input.read_bit(j, map_input.get_row_size() - 1 - i) == HARD_HIGHWAY) // set the color of hardTraverse highways cells to sky blue
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
				else if (map_input.read_bit(j, map_input.get_row_size() - 1 - i) == UNBLOCKED_HIGHWAY) // set the color of unblocked highways cells to blue
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
				map_image.at<cv::Vec3b>((map_input.get_row_size() - 1 - map_input.get_row_start()) * amplify_num + k, map_input.get_col_start()*amplify_num + h)[0] = COLOR_START[2];
				map_image.at<cv::Vec3b>((map_input.get_row_size() - 1 - map_input.get_row_start()) * amplify_num + k, map_input.get_col_start()*amplify_num + h)[1] = COLOR_START[1];
				map_image.at<cv::Vec3b>((map_input.get_row_size() - 1 - map_input.get_row_start()) * amplify_num + k, map_input.get_col_start()*amplify_num + h)[2] = COLOR_START[0];
			}
		}
		for (k = 0; k < amplify_num; ++k)
		{
			for (h = 0; h < amplify_num; ++h)
			{
				map_image.at<cv::Vec3b>((map_input.get_row_size() - 1 - map_input.get_row_goal()) * amplify_num + k, map_input.get_col_goal() * amplify_num + h)[0] = COLOR_GOAL[2];
				map_image.at<cv::Vec3b>((map_input.get_row_size() - 1 - map_input.get_row_goal()) * amplify_num + k, map_input.get_col_goal() * amplify_num + h)[1] = COLOR_GOAL[1];
				map_image.at<cv::Vec3b>((map_input.get_row_size() - 1 - map_input.get_row_goal()) * amplify_num + k, map_input.get_col_goal() * amplify_num + h)[2] = COLOR_GOAL[0];
			}
		}
		

		return map_image;
	}

	// display the map_input as an image
	cv::Mat map_maker::show_map_img()
	{
		cv::Mat new_map;
		new_map = show_map_img(map_output);
		return new_map;
	}

	// store the map_output into an image file in the path given by dest
	void map_maker::write_img_to_disk(char* dest)
	{
		cv::Mat mat_img = show_map_img();
		cv::imwrite(dest, mat_img);
	}

	void map_maker::reset_map_start_goal(grid_map &mapinput)
	{
		int start_col, start_row;
		int goal_col, goal_row;
		// Decide the start cell and the goal cell
		// start cell
		do
		{
			start_col = 1 + rand() % (mapinput.get_col_size() - 2);
			start_row = 1 + rand() % (mapinput.get_row_size() - 2);
		} while ((start_col>20 && start_col<(mapinput.get_col_size() - 20) && start_row>20 && start_row<(mapinput.get_row_size() - 20)) || !mapinput.check_available(start_col, start_row));
		// goal cell
		do
		{
			goal_col = 1 + rand() % (mapinput.get_col_size() - 2);
			goal_row = 1 + rand() % (mapinput.get_row_size() - 2);
		} while ((goal_col>20 && goal_col<(mapinput.get_col_size() - 20) && goal_row>20 && goal_row<(mapinput.get_row_size() - 20)) ||\
			!mapinput.check_available(goal_col, goal_row) || abs(goal_row - start_row) + abs(goal_col - start_col)<100);
		mapinput.set_start_cell(start_col, start_row);
		mapinput.set_goal_cell(goal_col, goal_row);

	}

	// read a text file in path given by src and store the map into map_loaded
	void map_maker::read_text(char* src, grid_map &map_loaded)
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
			map_loaded.col_rand_highway[i].clear();
			map_loaded.row_rand_highway[i].clear();
		}

		fin >> row_start >> comma >> col_start;
		map_loaded.set_start_cell(col_start, row_start);

		fin >> row_goal >> comma >> col_goal;
		map_loaded.set_goal_cell(col_goal, row_goal);

		for (i = 0; i < 8; ++i)
		{
			fin >> hardTraverse[1] >> comma >> hardTraverse[0];
			map_loaded.set_hardTraverse_cell(hardTraverse[0], hardTraverse[1], i);
		}

		fin >> row_num >> comma >> col_num;
		map_loaded.set_size(col_num, row_num);

		for (i = map_loaded.get_row_size() - 1; i >= 0; --i) {
			for (j = 0; j < map_loaded.get_col_size(); ++j)
			{
				fin >> bit_type;
				if (bit_type == ',')
					j -= 1;
				else
				{
					map_loaded.set_bit(j, i, bit_type);
					if (bit_type == UNBLOCKED_HIGHWAY || bit_type == HARD_HIGHWAY)
					{
						fin >> highway_num;
						map_loaded.row_rand_highway[highway_num].push_back(j);
						map_loaded.col_rand_highway[highway_num].push_back(i);
					}

				}
				
			}
		}
	}

	// show a window with f,g,h if click the map
	void map_maker::mouse_event(int event, int x, int y, int flags, void* param)
	{
		if (event == CV_EVENT_LBUTTONDBLCLK)
		{
			int h_num = 0;
			int k;
			std::ifstream fin;
			fin.open("f_g_h.txt");
			fin >> h_num;
			cv::Point loca_org(50, 25);
			cv::namedWindow("cell_information");
			cv::Mat map_image(800, 500, CV_8UC3, cv::Scalar::all(255));
			int amplify_num = 5;


			for (k = 0; k < h_num; ++k) {
				char coord[100]; // coordination of mouse in the cell
				double f_data[160][120];
				double g_data[160][120];
				double h_data[160][120];
				
				int i, j;
				int h_name;

				cv::Point f_org(50, 50 + k * 90);
				cv::Point g_org(50, 80 + k * 90);
				cv::Point h_org(50, 110 + k * 90);
				cv::Point line_left_org(25, 30 + k * 90);
				cv::Point line_right_org(475, 30 + k * 90);

				// read data from f_g_h.txt
				fin >> h_name;
				for (i = 0; i < 120; ++i)
					for (j = 0; j < 160; ++j)
					{
						fin >> f_data[j][i];
						fin >> g_data[j][i];
						fin >> h_data[j][i];
					}

				if (k == 0)
				{
					// coordination
					sprintf(coord, "col=%d, row=%d", int(floor(x / amplify_num)), 120 - int(floor(y / amplify_num)));
					cv::putText(map_image, coord, loca_org, CV_FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar::all(0), 2, 8);
					cv::line(map_image, line_left_org, line_right_org,cv::Scalar::all(0),2);
				}
				// f value
				sprintf(coord, "f%d(x) =%.3f", h_name, f_data[int(floor(x / amplify_num))][120 - int(floor(y / amplify_num))]);
				cv::putText(map_image, coord, f_org, CV_FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar::all(0), 2, 8);
				// g value
				sprintf(coord, "g%d(x) = %.3f", h_name, g_data[int(floor(x / amplify_num))][120 - int(floor(y / amplify_num))]);
				cv::putText(map_image, coord, g_org, CV_FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar::all(0), 2, 8);
				//h value
				sprintf(coord, "h%d(x) = %.3f", h_name, h_data[int(floor(x / amplify_num))][120 - int(floor(y / amplify_num))]);
				cv::putText(map_image, coord, h_org, CV_FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar::all(0), 2, 8);
				//line
				cv::line(map_image, line_left_org, line_right_org, cv::Scalar::all(0), 2);
			}

			
			cv::imshow("cell_information", map_image);
			fin.close();
		}

	}

	// read a text file in path given by src and store the map into map_loaded
	void map_maker::read_text(char* src)
	{
		read_text(src, map_output);
	}
}

