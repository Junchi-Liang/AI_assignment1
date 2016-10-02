#include "util.h"

namespace util_ns
{
	my_hash::my_hash(int size_of_columns, int size_of_rows)
	{
		int i;

		n_columns = size_of_columns;
		m_rows = size_of_rows;
		hash_bit = new bool[n_columns * m_rows + 5];
		for (i = 0; i < n_columns * m_rows; i++)
			hash_bit[i] = false;
	}

	my_hash::~my_hash()
	{
		delete[] hash_bit;
		hash_bit = NULL;
	}

	// hash function
	int my_hash::hash_index(int position_col, int position_row)
	{
		return position_row * n_columns + position_col;
	}

	// insert the cell (position_col, position_row) into the hash
	bool my_hash::insert(int position_col, int position_row)
	{
		if (position_col < 0 || position_col >= n_columns || position_row < 0 || position_row >= m_rows)
			return false;
		hash_bit[hash_index(position_col, position_row)] = true;
		return true;
	}

	// remove the cell (position_col, position_row) from the hash
	bool my_hash::remove(int position_col, int position_row)
	{
		if (position_col < 0 || position_col >= n_columns || position_row < 0 || position_row >= m_rows)
			return false;
		hash_bit[hash_index(position_col, position_row)] = false;
		return true;
	}

	// check if the cell (position_col, position_row) exists in the hash
	bool my_hash::exist(int position_col, int position_row)
	{
		if (position_col < 0 || position_col >= n_columns || position_row < 0 || position_row >= m_rows)
			return false;
		return hash_bit[hash_index(position_col, position_row)];
	}

}
