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

}

