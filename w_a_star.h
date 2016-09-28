#ifndef W_A_STAR_H
#define W_A_STAR_H

#include "abstract_solver.h"

namespace solver_ns
{
	class w_a_star: public abstract_solver
	{
		protected:
			double w;

			// return the f-value for the postion (cur_col, cur_row) while the goal is the (goal_col, goal_row) and the input map is map_input
                        virtual double f(double g, int cur_col, int cur_row, int goal_col, int goal_row, const map_maker_ns::grid_map &map_input)
			{
				return g + w * heuristic->h(cur_col, cur_row, goal_col, goal_row, map_input);
			}

		public:
			w_a_star(heuristic_ns::abstract_heuristic *heuristic_input, double w_input) : abstract_solver(heuristic_input)
			{
				w = w_input;
			}

	};

}

#endif
