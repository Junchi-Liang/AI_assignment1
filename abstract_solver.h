#ifndef ABSTRACT_SOLVER_H
#define ABSTRACT_SOLVER_H

#include <iostream>
#include <cstdio>
#include <cstdlib>

#include "abstract_heuristic.h"
#include "util.h"

namespace solver_ns
{
	class abstract_solver
	{
		protected:
			heuristic_ns::abstract_heuristic *heuristic;
			util_ns::my_heap *fringe;
			util_ns::my_hash *closed_list;
			int *trace_back_col, *trace_back_row;
			int n_columns, m_rows;
			double *f_table, *g_table, *h_table;

			// return the f-value for the postion (cur_col, cur_row) while the goal is the (goal_col, goal_row) and the input map is map_input
			virtual double f(double g, int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input) = 0;

			// initialize with a map of size (size_of_columns * size_of_rows)
			virtual void init(const map_maker_ns::grid_map &map_input, int size_of_columns, int size_of_rows)
			{
				n_columns = size_of_columns;
				m_rows = size_of_rows;
				if (fringe != NULL)
					delete fringe;
				if (closed_list != NULL)
					delete closed_list;
				if (trace_back_col != NULL)
					delete[] trace_back_col;
				if (trace_back_row != NULL)
					delete[] trace_back_row;
				if (h_table != NULL)
					delete[] h_table;
				if (g_table != NULL)
					delete[] g_table;
				if (f_table != NULL)
					delete[] f_table;
				
				fringe = new util_ns::my_heap(size_of_columns, size_of_rows);
				closed_list = new util_ns::my_hash(size_of_columns, size_of_rows);
				trace_back_col = new int[n_columns * m_rows + 5];
				trace_back_row = new int[n_columns * m_rows + 5];
				f_table = new double[n_columns * m_rows + 5];
				g_table = new double[n_columns * m_rows + 5];
				h_table = new double[n_columns * m_rows + 5];
				for (int i = 0; i < n_columns * m_rows; i++)
				{
					f_table[i] = -1;
					g_table[i] = -1;
					h_table[i] = -1;
				}
			}

			// read the trace back value
			virtual void read_trace_back(int position_col, int position_row, int &trace_column, int &trace_row)
			{
				trace_column = trace_back_col[position_row * n_columns + position_col];
				trace_row = trace_back_row[position_row * n_columns + position_col];
			}

			// set the trace back value
			virtual void set_trace_back(int position_col, int position_row, int trace_column, int trace_row)
			{
				trace_back_col[position_row * n_columns + position_col] = trace_column;
				trace_back_row[position_row * n_columns + position_col] = trace_row;
			}

			// build the path from (start_col, start_row) to (goal_col, goal_row) according to trace back, store the result to output
			virtual void build_result(int start_col, int start_row, int goal_col, int goal_row, map_maker_ns::result_path &output);

			virtual void set_f_table(int position_col, int position_row, double f_input)
			{
				f_table[position_row * n_columns + position_col] = f_input;
			}

			virtual double get_f_table(int position_col, int position_row)
			{
				return f_table[position_row * n_columns + position_col];
			}

			virtual void set_g_table(int position_col, int position_row, double g_input)
			{
				g_table[position_row * n_columns + position_col] = g_input;
			}

			virtual double get_g_table(int position_col, int position_row)
			{
				return g_table[position_row * n_columns + position_col];
			}

			virtual void set_h_table(int position_col, int position_row, double h_input)
			{
				h_table[position_row * n_columns + position_col] = h_input;
			}

			virtual double get_h_table(int position_col, int position_row)
			{
				return h_table[position_row * n_columns + position_col];
			}

			
			
		public:
			abstract_solver(heuristic_ns::abstract_heuristic *heuristic_input)
			{
				fringe = NULL;
				closed_list = NULL;
				trace_back_col = NULL;
				trace_back_row = NULL;
				heuristic = heuristic_input;
				h_table = NULL;
				g_table = NULL;
				f_table = NULL;
			}
			~abstract_solver()
			{
				heuristic = NULL;
				if (fringe != NULL)
					delete fringe;
				fringe = NULL;
				if (closed_list != NULL)
					delete closed_list;
				closed_list = NULL;
				if (trace_back_col != NULL)
					delete[] trace_back_col;
				trace_back_col = NULL;
				if (trace_back_row != NULL)
					delete[] trace_back_row;
				trace_back_row = NULL;
				if (h_table != NULL)
					delete[] h_table;
				h_table = NULL;
				if (g_table != NULL)
					delete[] g_table;
				g_table = NULL;
				if (f_table != NULL)
					delete[] f_table;
				f_table = NULL;
			}
			virtual void set_heuristic(heuristic_ns::abstract_heuristic *heuristic_input)
			{
				heuristic = heuristic_input;
			}
			
			// the interface for solving the problem where the input map is map_input, the start point is (start_col, start_row) and the goal point is (goal_col, goal_row)
			// return true when a path exist, otherwise return false
			// if a path exists, path_output stores the output path and expanded_nodes store the number of expanded nodes
			virtual bool solve(int start_col, int start_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input, map_maker_ns::result_path &path_output, int &expanded_nodes);

			// print f, g, h
			virtual void store_table(char* file_path)
			{
				std::ofstream fout;
				fout.open(file_path);
				int i, j;
				for (i = 0; i < m_rows; i++)
					for (j = 0; j < n_columns; j++)
						if (j == n_columns - 1)
							fout << get_f_table(j, i) << " " <<get_g_table(j, i) << " " << get_h_table(j, i) << std::endl;
						else
							fout << get_f_table(j, i) << " " << get_g_table(j, i) << " " << get_h_table(j, i) << " ";
				fout.close();
			}
			
	};
};

#endif

