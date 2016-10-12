#include "util.h"

namespace util_ns
{
	my_heap::my_heap(int size_of_columns, int size_of_rows)
	{
		int i;
		n_columns = size_of_columns;
		m_rows = size_of_rows;

		capacity = n_columns * m_rows + 5;
		size = 0;

		heap_col = new int[capacity];
		heap_row = new int[capacity];
		heap_f = new double[capacity];
		heap_g = new double[capacity];

		heap_index = new int[capacity];
		for (i = 0; i < capacity; i++)
			heap_index[i] = -1;
	}

	my_heap::~my_heap()
	{
		delete[] heap_col;
		heap_col = NULL;
		delete[] heap_row;
		heap_row = NULL;
		delete[] heap_f;
		heap_f = NULL;
		delete[] heap_g;
		heap_g = NULL;
		delete[] heap_index;
		heap_index = NULL;
	}

	// return true when the heap is empty
	bool my_heap::empty()
	{
		return (size == 0);
	}

	// check if the cell (position_col, position_row) exists in the heap, return true if exists
	bool my_heap::exist(int position_col, int position_row)
	{
		if (position_col < 0 || position_col >= n_columns || position_row < 0 || position_row >= m_rows)
			return false;
		return (heap_index[position_row * n_columns + position_col] > 0);
	}

	// swap node a and node b
	bool my_heap::swap(int a, int b)
	{
		int a_col = heap_col[a], a_row = heap_row[a], b_col = heap_col[b], b_row = heap_row[b];
		double tmp_double;
		heap_col[a] = b_col;
		heap_row[a] = b_row;
		heap_col[b] = a_col;
		heap_row[b] = a_row;
		tmp_double = heap_f[a];
		heap_f[a] = heap_f[b];
		heap_f[b] = tmp_double;
		tmp_double = heap_g[a];
		heap_g[a] = heap_g[b];
		heap_g[b] = tmp_double;

		heap_index[a_row * n_columns + a_col] = b;
		heap_index[b_row * n_columns + b_col] = a;
		return true;
	}

	// return true when a is better than b
	bool my_heap::better(double f_a, double g_a, double f_b, double g_b)
	{
		if (f_b < f_a)
			return false;
		if (f_a < f_b)
			return true;
		return (g_a < g_b);
	}
	
	// return true when node a is better than node b
	bool my_heap::compare(int a, int b)
	{
		return better(heap_f[a], heap_g[a], heap_f[b], heap_g[b]);
	}
	
	// move the node in order to maintain the heap, return final postion of this node
	int my_heap::move(int node)
	{
		int res = node;
		// go up
		while (res > 1 && compare(res, (res >> 1)))
		{
			swap(res, (res >> 1));
			res = (res >> 1);
		}
		// go down
		while ((res << 1) <= size)
		{
			int left_child = (res << 1), right_child = 1 + (res << 1), next_pos;
			if (right_child > size || compare(left_child, right_child))
				next_pos = left_child;
			else
				next_pos = right_child;
			if (compare(res, next_pos))
				break;
			swap(res, next_pos);
			res = next_pos;
		}
		return res;
	}

	// insert the cell (position_col, position_row) with f-value as f and g-value as g to the heap
	bool my_heap::insert(int position_col, int position_row, double f, double g)
	{
		if (position_col < 0 || position_col >= n_columns || position_row < 0 || position_row >= m_rows)
			return false;
		size++;
		heap_col[size] = position_col;
		heap_row[size] = position_row;
		heap_f[size] = f;
		heap_g[size] = g;
		heap_index[position_row * n_columns + position_col] = size;

		move(size);
		return true;
	}

	// remove the cell (position_col, position_row) from the heap
	bool my_heap::remove(int position_col, int position_row)
	{
		if (position_col < 0 || position_col >= n_columns || position_row < 0 || position_row >= m_rows)
			return false;
		int node = heap_index[position_row * n_columns + position_col];
		swap(node, size);
		size--;
		heap_index[position_row * n_columns + position_col] = -1;

		move(node);
		return true;
	}

	// get the root node without removing it
	bool my_heap::get_best(int &position_col, int &position_row, double &f, double &g)
	{
		if (size < 1)
			return false;
		position_col = heap_col[1];
		position_row = heap_row[1];
		f = heap_f[1];
		g = heap_g[1];
	}

	// pop the root of the heap, return its information
	bool my_heap::pop(int &position_col, int &position_row, double &f, double &g)
	{
		if (size < 1)
			return false;
		position_col = heap_col[1];
		position_row = heap_row[1];
		f = heap_f[1];
		g = heap_g[1];
		heap_index[position_row * n_columns + position_col] = -1;

		swap(1, size);
		size--;
		move(1);
		return true;
	}

	// update the cell (position_col, position_row) with f-value as f and g-value as g
	bool my_heap::update(int position_col, int position_row, double f, double g)
	{
		if (position_col < 0 || position_col >= n_columns || position_row < 0 || position_row >= m_rows)
			return false;
		int node = heap_index[position_row * n_columns + position_col];
		heap_f[node] = f;
		heap_g[node] = g;
		move(node);
		return true;
	}

	// get the f and g value
	bool my_heap::get_value(int position_col, int position_row, double &f, double &g)
	{
		if (!exist(position_col, position_row))
			return false;
		f = heap_f[heap_index[position_row * n_columns + position_col]];
		g = heap_g[heap_index[position_row * n_columns + position_col]];
		return true;
	}

}



