#ifndef INTEGRATED_A_STAR_H
#define INTEGRATED_A_STAR_H

#include "abstract_solver.h"

namespace solver_ns
{

	class integrated_a_star: public abstract_solver
	{
		private:
			int n_heuristic;
			heuristic_ns::abstract_heuristic **heuristic_list;
			util_ns::my_heap **fringe_set;
			util_ns::my_hash *closed_anchor, *closed_inad, *expanded_set;
			double **f_set, **h_set;
			double w1, w2;
			
			double f(double g, int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input)
			{
				return 0;
			}

			 // return the f-value for the postion (cur_col, cur_row) while the goal is the (goal_col, goal_row) and the input map is map_input
			double f(double g, int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input, int heuristic_index)
			{
				return g + w1 * (heuristic_list[heuristic_index])->h(cur_col, cur_row, goal_col, goal_row, map_input);
			}

			void clear_integrated();

			// return 1d index for 2d position
			int index_2dto1d(int position_col, int position_row)
			{
				return position_row * n_columns + position_col;
			}

			void set_f_set(int index_heuristic, int position_col, int position_row, double f_input)
			{
				f_set[index_heuristic][index_2dto1d(position_col, position_row)] = f_input;
			}

			double get_f_set(int index_heuristic, int position_col, int position_row)
			{
				return f_set[index_heuristic][index_2dto1d(position_col, position_row)];
			}

			void set_h_set(int index_heuristic, int position_col, int position_row, double h_input)
			{
				h_set[index_heuristic][index_2dto1d(position_col, position_row)] = h_input;
			}

			double get_h_set(int index_heuristic, int position_col, int position_row)
			{
				return h_set[index_heuristic][index_2dto1d(position_col, position_row)];
			}

			void init_integrated(const map_maker_ns::grid_map &map_input, int size_of_columns, int size_of_rows);

			void expand_state(int position_col, int position_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input);

		public:	
			integrated_a_star(int heuristic_size_input, double w1_input, double w2_input) : abstract_solver(NULL)
			{
				w1 = w1_input;
				w2 = w2_input;
				n_heuristic = heuristic_size_input;
				heuristic_list = new heuristic_ns::abstract_heuristic*[1 + n_heuristic];
				fringe_set = NULL;
				closed_anchor = NULL;
				closed_inad = NULL;
				expanded_set = NULL;
				f_set = NULL;
				h_set = NULL;
			}

			~integrated_a_star()
			{
				clear_integrated();
				delete[] heuristic_list;
				heuristic_list = NULL;
			}

			void set_heuristic(int index_heuristic, heuristic_ns::abstract_heuristic *heuristic_input)
			{
				heuristic_list[index_heuristic] = heuristic_input;
			}

			void set_w1(double w1_input)
			{
				w1 = w1_input;
			}

			void set_w2(double w2_input)
			{
				w2 = w2_input;
			}

			// the interface for solving the problem where the input map is map_input, the start point is (start_col, start_row) and the goal point is (goal_col, goal_row)
			// return true when a path exist, otherwise return false
			// if a path exists, path_output stores the output path and 
			// expanded_nodes store the number of expanded nodes (without re-count), and overall_expanded store number of expanded nodes (with re-count)
			bool solve(int start_col, int start_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input, map_maker_ns::result_path &path_output, int &expanded_nodes, int &overall_expanded);

			// print f, g, h
			void store_table(char* file_path, int index_admissible, const std::vector<int> &list_heuristic)
			{
				std::ofstream fout;
				fout.open(file_path);
				int i, j, k, l;
				
				fout << 1 + list_heuristic.size() << std::endl;
				fout << index_admissible << std::endl;
				for (i = 0; i < m_rows; i++)
					for (j = 0; j < n_columns; j++)
						if (j == n_columns - 1)
							fout << get_f_set(0, j, i) << " " << get_g_table(j, i) << " " << get_h_set(0, j, i) << std::endl;
						else
							fout << get_f_set(0, j, i) << " " << get_g_table(j, i) << " " << get_h_set(0, j, i) << " ";
				for (k = 0; k < list_heuristic.size(); k++)
				{
					l = list_heuristic[k];
					fout << l << std::endl;
					for (i = 0; i < m_rows; i++)
						for (j = 0; j < n_columns; j++)
							if (j == n_columns - 1)
								fout << get_f_set(k, j, i) << " " << get_g_table(j, i) << " " << get_h_set(k, j, i) << std::endl;
							else
								fout << get_f_set(k, j, i) << " " << get_g_table(j, i) << " " << get_h_set(k, j, i) << " ";
				}
				fout.close();
			}
	};

}

#endif
