#include "integrated_a_star.h"

namespace solver_ns
{
	void integrated_a_star::clear_integrated()
	{
		int i;
		if (fringe_set != NULL)
		{
			for (i = 0; i <= n_heuristic; i++)
			{
				delete fringe_set[i];
				fringe_set[i] = NULL;
			}
			delete[] fringe_set;
		}
		fringe_set = NULL;
		if (closed_anchor != NULL)
			delete closed_anchor;
		closed_anchor = NULL;
		if (closed_inad != NULL)
			delete closed_inad;
		closed_inad = NULL;
		if (expanded_set != NULL)
			delete expanded_set;
		expanded_set = NULL;
		if (f_set != NULL)
		{
			for (i = 0; i <= n_heuristic; i++)
			{
				delete f_set[i];
				f_set[i] = NULL;
			}
			delete[] f_set;
		}
		f_set = NULL;
		if (h_set != NULL)
		{
			for (i = 0; i <= n_heuristic; i++)
			{
				delete h_set[i];
				h_set[i] = NULL;
			}
			delete[] h_set;
		}
		h_set = NULL;
		if (g_table != NULL)
			delete[] g_table;
		g_table = NULL;
	}

	void integrated_a_star::init_integrated(const map_maker_ns::grid_map &map_input, int size_of_columns, int size_of_rows)
	{
		int i, j, cell_size = 5 + size_of_columns * size_of_rows;
		n_columns = size_of_columns;
		m_rows = size_of_rows;
		clear_integrated();
		if (trace_back_col != NULL)
			delete[] trace_back_col;
		if (trace_back_row != NULL)
			delete[] trace_back_row;
		trace_back_col = new int[cell_size];
		trace_back_row = new int[cell_size];
		fringe_set = new util_ns::my_heap*[1 + n_heuristic];
		closed_anchor = new util_ns::my_hash(size_of_columns, size_of_rows);
		closed_inad = new util_ns::my_hash(size_of_columns, size_of_rows);
		expanded_set = new util_ns::my_hash(size_of_columns, size_of_rows);
		f_set = new double*[1 + n_heuristic];
		h_set = new double*[1 + n_heuristic];
		g_table = new double[cell_size];
		for (i = 0; i <= n_heuristic; i++)
		{
			fringe_set[i] = new util_ns::my_heap(size_of_columns, size_of_rows);
			f_set[i] = new double[cell_size];
			h_set[i] = new double[cell_size];
			for (j = 0; j < cell_size; j++)
			{
				f_set[i][j] = -1;
				h_set[i][j] = -1;
				g_table[j] = -1;
			}
		}
	}

	void integrated_a_star::expand_state(int position_col, int position_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input)
	{
		int i, j;

		for (i = 0; i <= n_heuristic; i++)
			if ((fringe_set[i])->exist(position_col, position_row))
			{
				(fringe_set[i])->remove(position_col, position_row);
			}
		for (i = 0; i < map_maker_ns::MAX_DIR; i++)
		{
			int next_col = position_col + map_maker_ns::DELTA_COL[i];
			int next_row = position_row + map_maker_ns::DELTA_ROW[i];
			if (map_input.check_available(next_col, next_row))
			{
				double g_old = get_g_table(next_col, next_row); 
				double edge_cost = map_maker_ns::result_path::moving_cost(position_col, position_row, next_col, next_row, map_input);
				double g_new = get_g_table(position_col, position_row) + edge_cost;
				if (g_old < 0 || g_new < g_old)
				{
					set_g_table(next_col, next_row, g_new);
					set_trace_back(next_col, next_row, position_col, position_row);
					if (!closed_anchor->exist(next_col, next_row))
					{
						double f0 = f(g_new, next_col, next_row, goal_col, goal_row, map_input, 0);
						if (!(fringe_set[0])->exist(next_col, next_row))
							(fringe_set[0])->insert(next_col, next_row, f0, g_new);
						else
							(fringe_set[0])->update(next_col, next_row, f0, g_new);
						if (!closed_inad->exist(next_col, next_row))
						{
							for (i = 1; i <= n_heuristic; i++)
							{
								double fi = f(g_new, next_col, next_row, goal_col, goal_row, map_input, i);
								if (fi <= w2 * f0)
								{
									if (!(fringe_set[i])->exist(next_col, next_row))
										(fringe_set[i])->insert(next_col, next_row, fi, g_new);
									else
										(fringe_set[i])->update(next_col, next_row, fi, g_new);
								}
							}
						}
					}
				}
			}
		}
	}

	// the interface for solving the problem where the input map is map_input, the start point is (start_col, start_row) and the goal point is (goal_col, goal_row)
	// return true when a path exist, otherwise return false
	// if a path exists, path_output stores the output path and
	// expanded_nodes store the number of expanded nodes (without re-count), and overall_expanded store number of expanded nodes (with re-count)
	bool integrated_a_star::solve(int start_col, int start_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input, map_maker_ns::result_path &path_output, int &expanded_nodes, int &overall_expanded)
	{
		int map_size_col, map_size_row, i, cur_col_0, cur_row_0, cur_col_i, cur_row_i;
		bool has_element_0 = true;

		map_size_col = map_input.get_col_size();
		map_size_row = map_input.get_row_size();
		overall_expanded = 0;
		expanded_nodes = 0;

		init_integrated(map_input, map_size_col, map_size_row);

		set_g_table(start_col, start_row, 0);
		for (i = 0; i < n_heuristic; i++)
		{
			double fi = f(0, start_col, start_row, goal_col, goal_row, map_input, i);
			(fringe_set[i])->insert(start_col, start_row, fi, 0);
		}

		while (has_element_0)
		{
			double f0, g0, fi, gi;
			for (i = 1; has_element_0 && i <= n_heuristic; i++)
			{
				has_element_0 = (fringe_set[0])->get_best(cur_col_0, cur_row_0, f0, g0);
				if (!has_element_0)
					break;
				bool has_element = (fringe_set[i])->get_best(cur_col_i, cur_row_i, fi, gi);
				if (has_element && fi <= w2 * f0)
				{
					if (cur_col_i ==  goal_col && cur_row_i == goal_row)
					{
						build_result(start_col, start_row, goal_col, goal_row, path_output);
						return true;
					}
					overall_expanded++;
					if (!expanded_set->exist(cur_col_i, cur_row_i))
					{
						expanded_set->insert(cur_col_i, cur_row_i);
						expanded_nodes++;
					}
					expand_state(cur_col_i, cur_row_i, goal_col, goal_row, map_input);
					closed_inad->insert(cur_col_i, cur_row_i);
				}
				else
				{
					if (cur_col_0 == goal_col && cur_row_0 == goal_row)
					{
						build_result(start_col, start_row, goal_col, goal_row, path_output);
						return true;
					}
					overall_expanded++;
					if (!expanded_set->exist(cur_col_0, cur_row_0))
					{
						expanded_set->insert(cur_col_0, cur_row_0);
						expanded_nodes++;
					}
					expand_state(cur_col_0, cur_row_0, goal_col, goal_row, map_input);
					closed_anchor->insert(cur_col_0, cur_row_0);
				}
			}
		}

		return false;
	}
	
}











