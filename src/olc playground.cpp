#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include "fparser.hh"
#include <thread>
#include <queue>
#include <mutex>

std::mutex mtx;

class graph_draw : public olc::PixelGameEngine
{
public:
	graph_draw()
	{
		sAppName = "Example";
	}

public:

	/*int x_right_limit = -100;
	int x_left_limit = 100;
	int y_down_limit = -100;
	int y_up_limit = 100;*/

	double zoom = 2;
	double epsilon = 0.0001;
	double calculation_accuracy = 1;
	double h = 0.1;
	double x_for_point_to_find = 4;
	double min_val = -100;
	double max_val = 100;

	int32_t ms_test;

	const int move_to_center = 541;

	double x_offset = 0;
	double y_offset = 0;

	bool highlight_x_axis_crossing = false;
	bool moving_without_redrawing = false;
	bool show_cords_on_mouse = false;
	bool calc_derivative = false;
	bool draw_found_point = false;
	bool writing_point_data = false;
	bool calc_double_derivative = false;
	bool menu_drawing = false;
	bool writing_range = false;
	short writing_value_num = 0;

	bool writing_function_data = false;
	std::string function_string = "";
	std::string range_string = "";
	FunctionParser fparser;

	//for drawing with mouse
	std::vector<olc::vf2d> drawed_pixels;

	std::vector<olc::vf2d> points;

	void draw_plot()
	{
		//draw axises
		DrawLine({ 0, (int)((ScreenHeight() / 2) - y_offset) }, { ScreenWidth(), (int)((ScreenHeight() / 2) - y_offset) }); // x axis
		DrawLine({ (ScreenWidth() / 2) - (int)x_offset, 0 }, { (ScreenWidth() / 2) - (int)x_offset, ScreenHeight() });  // y axis

		DrawString({ ScreenWidth() - 360,0 }, "(OFFSET)(X: " + std::to_string(x_offset) + ")" + "Y: " + std::to_string(-1 * y_offset));
		DrawString({ ScreenWidth() - 360,20 }, "ZOOM: " + std::to_string(zoom));

		if (calc_double_derivative)
			DrawString({ ScreenWidth() - 360, 40}, "F\'\'(x)");
		else if (calc_derivative)
			DrawString({ ScreenWidth() - 360, 40}, "F\'(x)");
		else
			DrawString({ ScreenWidth() - 360, 40}, "F\(x)");

		if (highlight_x_axis_crossing)
			DrawString({ 0,2 }, "SHOW FUNCTION ROOTS APPROXIMATION epsilon = " + std::to_string(epsilon), olc::RED);

		DrawString({ 0, ScreenHeight() - 100 }, "CALCULATION ACCURACY: " + std::to_string(calculation_accuracy));

		if (show_cords_on_mouse)
		{
			if (!highlight_x_axis_crossing)
				DrawString({ 0,2 }, "SHOW COORDINATES ON CURSOR", olc::YELLOW);
			else
				DrawString({ 0,10 }, "SHOW COORDINATES ON CURSOR", olc::YELLOW);

			auto mouse_x = GetMouseX();
			auto mouse_y = GetMouseY();

			//draw axises
			DrawLine({ mouse_x, (int)((ScreenHeight() / 2) - y_offset) }, { ScreenWidth(), (int)((ScreenHeight() / 2) - y_offset) }); // x axis
			DrawLine({ (ScreenWidth() / 2) - (int)x_offset, mouse_y }, { (ScreenWidth() / 2) - (int)x_offset, ScreenHeight() });  // y axis
			DrawString({ mouse_x + 4, mouse_y + 4 }, "X: " + std::to_string((mouse_x - ScreenWidth() / 2 + x_offset) / zoom) + " Y: " + std::to_string((-1 * (mouse_y - ScreenHeight() / 2 + y_offset) / zoom)), olc::YELLOW);
		}

		if (draw_found_point)
		{
			if (!highlight_x_axis_crossing && !show_cords_on_mouse)
				DrawString({ 0,2 },  "LOOKING FOR POINT WITH X: " + writing_string, olc::GREEN);
			else if (highlight_x_axis_crossing && !show_cords_on_mouse)
				DrawString({ 0,10 }, "LOOKING FOR POINT WITH X: " + writing_string, olc::GREEN);
			else if (!highlight_x_axis_crossing && show_cords_on_mouse)
				DrawString({ 0,10 }, "LOOKING FOR POINT WITH X: " + writing_string, olc::GREEN);
			else if (highlight_x_axis_crossing && show_cords_on_mouse)
				DrawString({ 0,18 }, "LOOKING FOR POINT WITH X: " + writing_string, olc::GREEN);
		}
	}

	static double func(FunctionParser fparser, double i)
	{
		double* mass_d = new double[1];
		mass_d[0] = i;
		double val = fparser.Eval(mass_d);
		delete[] mass_d;
		return val;
	}

	void sort_points() 
	{
		for (size_t i = 0; i < points.size(); i++)
		{
			for (size_t j = i; j < points.size(); j++)
				if (points[i].x > points[j].x)
				{
					std::swap(points[i], points[j]);
				}
		}
		
	}

	void calculate_function() 
	{
		points.clear();

		double local_max_val = max_val;
		double local_min_val = min_val;
		double split		 = local_min_val >= 0? ceil((abs(local_max_val) - local_min_val)/2) : ceil(-1*(abs(local_max_val) - abs(local_min_val)) / 2);

		std::queue<std::pair<double,double>> calc_queue;

		std::pair<double, double> push_pair;

		push_pair.first		= local_min_val;
		push_pair.second	= local_min_val + split;

		calc_queue.push(push_pair);

		push_pair.first  = local_min_val + split;
		push_pair.second = local_max_val;

		calc_queue.push(push_pair);

		FunctionParser fp1;
		FunctionParser fp2;

		fp1.Parse(function_string, "x");
		fp2.Parse(function_string, "x");

		auto calc_thread = [=](std::pair<double,double> min_max, FunctionParser fp)
		{
			for (double i = min_max.first; i < min_max.second; i += 1 / calculation_accuracy / zoom)
			{
				double val;

				if (calc_derivative)
					val = (func(fp, i + h) - func(fp, i - h)) / (2 * h);
				else if (calc_double_derivative)
					val = (func(fp, i + h) - 2 * func(fp, i) + func(fp, i - h)) / (h * h);
				else
					val = func(fp, i);

				olc::vf2d v({ (float)i * (float)zoom + move_to_center,(float)((int)(ScreenHeight() / 2) - val * (float)zoom) });

				mtx.lock();
				points.push_back(v);
				mtx.unlock();
			}
		};

		while (calc_queue.size() != 0)
		{
			std::thread t1(calc_thread, calc_queue.front(), fp1);
			calc_queue.pop();
			std::thread t2(calc_thread, calc_queue.front(), fp2);
			calc_queue.pop();

			t1.join();
			t2.join();
		}
	}

	void draw_graph(double(*func)(FunctionParser, double))
	{
		draw_plot();
		draw_mouse_drawed();

		if (zoom <= 0)
			zoom = 1;

		double root_val = epsilon / zoom;

		for (size_t i = 0; i < points.size(); i++/zoom)
		{
			if ((points[i].x - x_offset<= ScreenWidth() || points[i].y - y_offset <= ScreenHeight()) && (points[i].x - x_offset >= 0 && points[i].y - y_offset >= 0))
			{
				if (!highlight_x_axis_crossing)
				{
					Draw(points[i] - olc::vf2d(x_offset, y_offset), olc::WHITE);
				}
				else
				{
					if (abs((points[i].y - ScreenHeight() / 2)/zoom) <= root_val)
						DrawCircle(points[i] - olc::vf2d(x_offset, y_offset), 2, olc::RED);
					else
						Draw(points[i] - olc::vf2d(x_offset, y_offset), olc::WHITE);
				}
			}
		}

		if (draw_found_point)
		{
			x_for_point_to_find = std::stod(writing_string);
			DrawCircle(find_point(x_for_point_to_find), 2, olc::GREEN);
		}
	}

	olc::vf2d find_point(double x)
	{
		double y;

		if (calc_derivative)
			y = (func(fparser, x + h) - func(fparser, x - h)) / (2 * h);
		else if (calc_double_derivative)
			y = (func(fparser, x + h) - 2 * func(fparser, x) + func(fparser, x - h)) / (h * h);
		else
			y = func(fparser, x);

		olc::vf2d v({ (float)x * (float)zoom + move_to_center - (float)x_offset,(float)((int)(ScreenHeight() / 2) - y * (float)zoom) - (float)y_offset });

		return v;
	}

	void draw_mouse_drawed()
	{
		for (size_t i = 0; i < drawed_pixels.size(); i += 1)
		{
			if (drawed_pixels[i].x - x_offset >= 0 && drawed_pixels[i].x - x_offset <= ScreenWidth() && drawed_pixels[i].y - y_offset >= 0 && drawed_pixels[i].y - y_offset <= ScreenHeight())
				Draw({ (int)(drawed_pixels[i].x - (float)x_offset),  (int)(drawed_pixels[i].y - (float)y_offset) }, olc::MAGENTA);
		}
	}

	bool OnUserCreate() override
	{
		function_string = "x^2";
		calculate_function();
		draw_graph(func);

		return true;
	}

	std::string writing_string = "";

	std::string inputs = "abcdefghijklmnopqrstuvwxyz0123456789 ";

	bool OnUserUpdate(float fElapsedTime) override
	{
		if (writing_range) 
		{
			if (GetKey(olc::Key::K0).bPressed)
			{
				range_string += std::to_string(0);
			}
			if (GetKey(olc::Key::K1).bPressed)
			{
				range_string += std::to_string(1);
			}
			if (!GetKey(olc::Key::SHIFT).bHeld && GetKey(olc::Key::K2).bPressed)
			{
				range_string += std::to_string(2);
			}
			if (GetKey(olc::Key::K3).bPressed)
			{
				range_string += std::to_string(3);
			}
			if (GetKey(olc::Key::K4).bPressed)
			{
				range_string += std::to_string(4);
			}
			if (GetKey(olc::Key::K5).bPressed)
			{
				range_string += std::to_string(5);
			}
			if (GetKey(olc::Key::K6).bPressed)
			{
				range_string += std::to_string(6);
			}
			if (GetKey(olc::Key::K7).bPressed)
			{
				range_string += std::to_string(7);
			}
			if (GetKey(olc::Key::K8).bPressed)
			{
				range_string += std::to_string(8);
			}
			if (GetKey(olc::Key::K9).bPressed)
			{
				range_string += std::to_string(9);
			}
			if (GetKey(olc::Key::CTRL).bPressed)
			{
				range_string += '.';
			}
			if (GetKey(olc::Key::BACK).bPressed && range_string.size() > 0)
				range_string.erase(range_string.end() - 1);

			if(GetKey(olc::Key::SHIFT).bHeld && GetKey(olc::Key::K2).bPressed)
				range_string += "-";

			Clear(olc::BLACK);

			if(writing_value_num == 0)
				DrawString({ 0,2 }, "BEGIN VALUE X: " + range_string);

			if (writing_value_num == 1)
				DrawString({ 0,2 }, "END VALUE X: " + range_string);

			if (GetKey(olc::Key::ENTER).bPressed)
			{
				writing_value_num++;

				if (writing_value_num == 1)
				{
					min_val = std::stod(range_string);
				}

				if (writing_value_num == 2)
				{
					max_val = std::stod(range_string);
					writing_value_num = 0;
					writing_range = false;

					Clear(olc::BLACK);
					calculate_function();
					draw_graph(func);
				}

				range_string = "";
			}

			return true;
		}

		if (writing_function_data)
		{
			for (size_t i = 1; i < 64; i++)
			{
				if (GetKey((olc::Key)i).bPressed) 
				{
					if (i == 55)
						continue;

					if (i == 57)
						break;

					if (i == 56)
						function_string += ".";

					if (i < 27)
						function_string += inputs[i-1];

					if (GetKey(olc::Key::SHIFT).bHeld) 
					{
						if (i >= 27 && i <= 37)
						{
							switch (i)
							{
							case 28:
								function_string += "!";
								break;
							case 29:
								function_string += "-";
								break;
							case 30:
								function_string += "+";
								break;
							case 33:
								function_string += "^";
								break;
							case 34:
								function_string += "/";
								break;
							case 35:
								function_string += "*";
								break;
							case 36:
								function_string += "(";
								break;
							case 27:
								function_string += ")";
								break;

							default:
								break;
							}
						}
					}
					else
					{
						if (i >= 27 && i <= 37)
						{
							function_string += inputs[i - 1];
						}
					}

					if (i == 63 && function_string.size() > 0)
						function_string.erase(function_string.end() -1);
				}
			}

			Clear(olc::BLACK);
			DrawString({ 0,2 }, "CALCULATING: F(X) = " + function_string);

			if (GetKey(olc::Key::INS).bPressed)
			{
				fparser.Parse(function_string, "x");

				writing_function_data = false;
				Clear(olc::BLACK);
				calculate_function();
				draw_graph(func);
			}

			return true;
		}

		if (writing_point_data)
		{
			if (GetKey(olc::Key::K0).bPressed)
			{
				writing_string += std::to_string(0);
			}
			if (GetKey(olc::Key::K1).bPressed)
			{
				writing_string += std::to_string(1);
			}
			if (GetKey(olc::Key::K2).bPressed)
			{
				writing_string += std::to_string(2);
			}
			if (GetKey(olc::Key::K3).bPressed)
			{
				writing_string += std::to_string(3);
			}
			if (GetKey(olc::Key::K4).bPressed)
			{
				writing_string += std::to_string(4);
			}
			if (GetKey(olc::Key::K5).bPressed)
			{
				writing_string += std::to_string(5);
			}
			if (GetKey(olc::Key::K6).bPressed)
			{
				writing_string += std::to_string(6);
			}
			if (GetKey(olc::Key::K7).bPressed)
			{
				writing_string += std::to_string(7);
			}
			if (GetKey(olc::Key::K8).bPressed)
			{
				writing_string += std::to_string(8);
			}
			if (GetKey(olc::Key::K9).bPressed)
			{
				writing_string += std::to_string(9);
			}
			if (GetKey(olc::Key::CTRL).bPressed)
			{
				writing_string += '.';
			}
			if (GetKey(olc::Key::BACK).bPressed && writing_string.size() > 0)
				writing_string.erase(writing_string.end() - 1);

			Clear(olc::BLACK);
			DrawString({ 0,2 }, "LOOKING FOR POINT WITH X: " + writing_string);

			if (GetKey(olc::Key::F).bPressed)
			{
				draw_found_point = true;
				writing_point_data = false;
				Clear(olc::BLACK);
				draw_graph(func);
			}

			return true;
		}

		if (GetKey(olc::Key::F).bPressed)
		{
			Clear(olc::BLACK);
			draw_found_point = false;
			draw_graph(func);
		}

		//LMB PRESS
		if (GetMouse(0).bHeld)
		{
			if (drawed_pixels.size() == 0)
			{
				drawed_pixels.push_back({ (float)((GetMouseX() + x_offset)) , (float)((GetMouseY() + y_offset)) });
				draw_mouse_drawed();
			}

			size_t sz = drawed_pixels.size();

			for (size_t i = 0; i < drawed_pixels.size(); i++)
			{
				if (drawed_pixels[i].x == GetMouseX() && drawed_pixels[i].y == GetMouseY())
				{
					break;
				}
				else if (i == sz - 1)
				{
					drawed_pixels.push_back({ (float)((GetMouseX() + x_offset)) , (float)((GetMouseY() + y_offset)) });
					draw_mouse_drawed();
				}
			}
		}

		if (GetMouse(1).bHeld)
		{
			std::vector<size_t> erase_list;

			for (size_t i = 0; i < drawed_pixels.size(); i++)
			{
				if (drawed_pixels[i].x - x_offset <= GetMouseX() + 4 && drawed_pixels[i].x - x_offset >= GetMouseX() - 4 && drawed_pixels[i].y - y_offset <= GetMouseY() + 4 && drawed_pixels[i].y - y_offset >= GetMouseY() - 4)
				{
					erase_list.push_back(i);
				}
			}

			size_t erased = 0;

			for (size_t i = 0; i < erase_list.size(); i++)
			{
				drawed_pixels.erase(drawed_pixels.begin() + erase_list[0] - erased++);
				erase_list.erase(erase_list.begin() + 0);
			}

			Clear(olc::BLACK);
			draw_graph(func);
		}

		if (moving_without_redrawing)
		{
			Clear(olc::BLACK);
			DrawString({ 0,2 }, "MOVING WITHOUT REDRAWING (X: " + std::to_string(x_offset) + ")" + "Y: " + std::to_string(-1 * y_offset) + ") ZOOM: " + std::to_string(zoom));

			if (GetKey(olc::Key::W).bHeld)
			{
				y_offset -= 0.5;
				Clear(olc::BLACK);
			}
			if (GetKey(olc::Key::S).bHeld)
			{
				y_offset += 0.5;
				Clear(olc::BLACK);
			}
			if (GetKey(olc::Key::A).bHeld)
			{
				x_offset -= 0.5;
				Clear(olc::BLACK);
			}
			if (!GetKey(olc::Key::SHIFT).bHeld && GetKey(olc::Key::D).bHeld)
			{
				x_offset += 0.5;
				Clear(olc::BLACK);
			}

			if(GetKey(olc::Key::SHIFT).bHeld && GetKey(olc::Key::D).bPressed)
			{
				writing_range = !writing_range;
			}

			if (GetMouseWheel() > 0)
			{
				zoom /= 2;
			}
			else if (GetMouseWheel() < 0)
			{
				zoom *= 2;
			}

			//Moving without redrawing stop
			if (GetKey(olc::Key::CTRL).bPressed)
			{
				moving_without_redrawing = false;
				Clear(olc::BLACK);
				draw_graph(func);
			}

			return true;
		}

		if (GetMouseWheel() > 0)
		{
			zoom /= 2;
			x_offset /= 2;
			y_offset /= 2;
			Clear(olc::BLACK);
			calculate_function();
			draw_graph(func);
		}
		else if (GetMouseWheel() < 0)
		{
			zoom *= 2;
			x_offset *= 2;
			y_offset *= 2;
			Clear(olc::BLACK);
			calculate_function();
			draw_graph(func);
		}

		if (!GetKey(olc::Key::SHIFT).bHeld)
		{
			if (GetKey(olc::Key::W).bHeld)
			{
				y_offset -= 0.5;
				Clear(olc::BLACK);
				draw_graph(func);
			}
			if (GetKey(olc::Key::S).bHeld)
			{
				y_offset += 0.5;
				Clear(olc::BLACK);
				draw_graph(func);
			}
			if (GetKey(olc::Key::A).bHeld)
			{
				x_offset -= 0.5;
				Clear(olc::BLACK);
				draw_graph(func);
			}
			if (GetKey(olc::Key::D).bHeld)
			{
				x_offset += 0.5;
				Clear(olc::BLACK);
				draw_graph(func);
			}

			if (GetKey(olc::Key::P).bPressed)
			{
				if (calc_double_derivative)
					calc_double_derivative = false;
				calc_derivative = !calc_derivative;
				calculate_function();
				Clear(olc::BLACK);
				draw_graph(func);
			}
		}
		else
		{
			if (GetKey(olc::Key::F).bPressed)
			{
				writing_string = "";
				writing_point_data = true;
			}

			if (GetKey(olc::Key::W).bHeld)
			{
				y_offset -= 2;
				Clear(olc::BLACK);
				draw_graph(func);
			}
			if (GetKey(olc::Key::S).bHeld)
			{
				y_offset += 2;
				Clear(olc::BLACK);
				draw_graph(func);
			}
			if (GetKey(olc::Key::A).bHeld)
			{
				x_offset -= 2;
				Clear(olc::BLACK);
				draw_graph(func);
			}
			if (GetKey(olc::Key::D).bHeld)
			{
				x_offset += 2;
				Clear(olc::BLACK);
				draw_graph(func);
			}

			if (GetKey(olc::Key::SPACE).bPressed)
			{
				writing_range = !writing_range;
			}

			if (GetKey(olc::Key::P).bPressed)
			{
				if (calc_derivative)
					calc_derivative = false;

				calc_double_derivative = !calc_double_derivative;
				calculate_function();
				Clear(olc::BLACK);
				draw_graph(func);
			}

			if (GetKey(olc::Key::INS).bPressed)
			{
				function_string = "";
				writing_function_data = true;
			}
		}

		if (GetKey(olc::Key::E).bHeld)
		{
			if (GetKey(olc::Key::UP).bPressed)
			{
				if (!GetKey(olc::Key::SHIFT).bHeld)
					epsilon += 0.0001;
				else
					epsilon += 0.001;
			}
			else if (GetKey(olc::Key::DOWN).bPressed)
			{
				if (!GetKey(olc::Key::SHIFT).bHeld)
					epsilon -= 0.0001;
				else
					epsilon -= 0.001;
			}

			Clear(olc::BLACK);
			draw_graph(func);
		}

		if (GetKey(olc::Key::Q).bHeld)
		{
			if (GetKey(olc::Key::UP).bPressed)
			{
				calculation_accuracy *= 2;
				calculate_function();
				Clear(olc::BLACK);
				draw_graph(func);
			}
			else if (GetKey(olc::Key::DOWN).bPressed)
			{
				calculation_accuracy /= 2;
				calculate_function();
				Clear(olc::BLACK);
				draw_graph(func);
			}
		}

		if (GetKey(olc::Key::U).bPressed)
		{
			Clear(olc::BLACK);
			draw_graph(func);
		}

		if (GetKey(olc::Key::C).bPressed && !GetKey(olc::Key::SHIFT).bHeld)
		{
			x_offset = 0;
			y_offset = 0;
			Clear(olc::BLACK);
			draw_graph(func);
		}
		else if(GetKey(olc::Key::C).bPressed)
		{
			calculate_function();
			Clear(olc::BLACK);
			draw_graph(func);
		}

		//Highlight function roots
		if (GetKey(olc::Key::R).bPressed)
		{
			highlight_x_axis_crossing = !highlight_x_axis_crossing;
			Clear(olc::BLACK);
			draw_graph(func);
		}

		//Moving without redrawing
		if (GetKey(olc::Key::CTRL).bPressed)
		{
			moving_without_redrawing = true;
		}

		//Show coordinates on mouses
		if (GetKey(olc::Key::M).bPressed)
		{
			show_cords_on_mouse = !show_cords_on_mouse;
			Clear(olc::BLACK);
			draw_graph(func);
		}

		if (GetKey(olc::Key::ESCAPE).bPressed)
		{
			menu_drawing = !menu_drawing;
		}

		return true;
	}
};


int main()
{
	graph_draw demo;
	if (demo.Construct(1080, 720, 1, 1))
		demo.Start();

	return 0;
}
