// ProjectDelta.cpp : Defines the entry point for the console application.
//
#include <iostream>
#include "LY_NN.h"
#include <vector>
#include <math.h>
#include <map>
#include <assert.h>
#include <random>
#include <ostream>
#include <time.h>
#include <string.h>
#include <fstream>
#include <limits>
#include <climits>
#include <cstdlib>
#include <algorithm>
#include <cmath>

using namespace std;
#define pi 3.14159265358979323846
#define ATRAND (double)rand() / RAND_MAX
#define timestep 0.2

class ship {
public:
	double x;
	double y;
	double theta;
	double omega;
	double v = 3.0;
	double dt = 0.2;
	double T = 5.0;
	double u;
	double xprev;
	double yprev;
	double thetaprev;
	double omegaprev;
	double xinit;
	double yinit;
	double thetainit;
	double omegainit;
	int stopcheck = 0;
	int OOB = 0;
	void init();
	void xcalc();
	void ycalc();
	void thetacalc();
	void omegacalc();
	//void endsimulation(int min, int max);
	void simulate();
	void reset();
	void hr1_test();
	void mr1test(double goalx, double goaly);
	void mr1reset();
	void hr4reset();
};

void ship::init() {
	x = (rand() % (600 - 400)) + 400;  				  // boat spawns between 400 and 600 x
	y = (rand() % (600 - 400)) + 400;  				  // boat spawns between 400 and 600 y
	double thetamax = pi;
	int thetamin = -pi;
	theta = ATRAND * (thetamax - thetamin) + thetamin;

	int omegamax = 10;
	int omegamin = 1;
	omega = ATRAND * (omegamax - omegamin) + omegamin;
	u = ATRAND*(30) - 15;
	xprev = x;
	yprev = y;
	thetaprev = theta;
	omegaprev = omega;
	xinit = x;
	yinit = y;
	thetainit = theta;
	omegainit = omega;
}

void ship::reset() {
	x = xinit;
	y = yinit;
	theta = thetainit;
	omega = omegainit;
	OOB = 0;
}

void ship::xcalc() {
	x = x + (v * sin(theta) * dt);
}

void ship::ycalc() {
	y = y + (v * cos(theta) * dt);
}

void ship::thetacalc() {
	theta = theta + (omega * dt);
}

void ship::omegacalc() {
	omega = omega + ((u - omega) * dt / T);
}

//void ship::endsimulation(int min, int max) {
//	if (x < min || y < min || x > max || y > max) {
//  	  cout << "beyond bounds. breaking" << endl;
//  	  stopcheck = 1;
//	}
//}

void ship::simulate() {
	xprev = x;
	yprev = y;
	thetaprev = theta;
	omegaprev = omega;
	omegacalc();
	thetacalc();
	ycalc();
	xcalc();
}

void ship::hr1_test() {
	x = 700;
	y = 950;
	xinit = 700;
	yinit = 950;
	theta = pi;
	thetainit = theta;
}

void ship::mr1test(double goalx, double goaly) {
	x = goalx - 10;
	y = goaly + 50;
	xinit = x;
	yinit = y;
	theta = pi/2;
	thetainit = pi/2;
	omega = 0;
	omegainit = 0;
}

void ship::mr1reset() {
	theta = pi/2;
	thetainit = pi/2;
	omega = 0;
	omegainit = 0;
}

void ship::hr4reset() {
	xinit = (rand() % (600 - 400)) + 400;  				  // boat spawns between 400 and 600 x
	yinit = (rand() % (600 - 400)) + 400;  				  // boat spawns between 400 and 600 y
	double thetamax = pi;
	int thetamin = -pi;
	thetainit = ATRAND * (thetamax - thetamin) + thetamin;

	int omegamax = 10;
	int omegamin = 1;
	omegainit = ATRAND * (omegamax - omegamin) + omegamin;
}

class goal {
public:
	double x1;
	double y1;
	double x2;
	double y2;
	void init(int max);
};

void goal::init(int max) {
	x1 = 900;
	y1 = 1000;
	x2 = 1000;
	y2 = 900;
}

class policy {
public:
	double fitness;
	int ts = 0;
	vector<double> weights;
	void init(int num_of_weights);
	vector<double> xpath;
	vector<double> ypath;
};

void policy::init(int num_of_weights) {
	vector<double> b;
	int temp = num_of_weights;
	for (int i = 0; i < temp; i++) {
		b.push_back(ATRAND - ATRAND);
	}
	weights = b;
}

int shutdown(ship& agent, double min, double max) {
	int oob = 0;
	if (agent.x < min || agent.y < min || agent.x > max || agent.y > max) {
		//cout << "beyond bounds. breaking" << endl;
		oob = agent.OOB = 1;
	}
	return oob;
}

//int pass_check(ship& agent, goal& goal) {
//    double x1;
//    double x2;
//    double x3;
//    double x4;
//    double y1;
//    double y2;
//    double y3;
//    double y4;
//    double m1;
//    double m2;
//    double b1;
//    double b2;
//    double xinter;
//    double yinter;
//    double check1;
//    double check2;
//    int pass;
//    x1 = agent.xprev;
//    x2 = agent.x;
//    x3 = goal.x1;
//    x4 = goal.x2;
//    y1 = agent.yprev;
//    y2 = agent.y;
//    y3 = goal.y1;
//    y4 = goal.y2;
//    m1 = (y2 - y1) / (x2 - x1);
//    m2 = (y4 - y3) / (x4 - x3);
//    b1 = y1 - m1 * x1;
//    b2 = y3 - m2 * x3;
//    xinter = (b2 - b1) / (m1 - m2);
//    yinter = (m1 * xinter) + b1;
//    if (x2 > xinter > x1 || x2 < xinter < x1) {
//   	 check1 = 1;
//    }
//    if (y2 > yinter > y1 || y2 < yinter < y1) {
//   	 check2 = 1;
//    }
//    if (check1 == check2) {
//   	 pass = 1;
//    }
//    if (check1 != check2) {
//   	 pass = 0;
//    }
//    return pass;
//}

int passcheckez(ship& agent, goal& goal) {
	int pass = 0;
	if (agent.x > goal.x1 && agent.y > goal.y2) {
		pass = 1;
		// MR 2
		assert(agent.x > goal.x1);
		assert(agent.y > goal.y2);
	}
	return pass;
}

double evaluate(ship boat, goal goal, vector<policy> population, int popsize, int sim) {
	double midpointx;
	double midpointy;
	double distance;
	midpointx = (goal.x1 + goal.x2) / 2;
	midpointy = (goal.y1 + goal.y2) / 2;
	distance = sqrt(pow(midpointx - boat.x, 2) + pow(midpointy - boat.y, 2));
	return distance;
}

vector<policy> down_select(int i_pop_size, vector<policy> pop) {
	vector<policy> ds_pop;
	assert(pop.size() == i_pop_size);
	while (ds_pop.size() < i_pop_size / 2) {
		int spot1 = rand() % i_pop_size;
		int spot2 = rand() % i_pop_size;
		while (spot2 == spot1) {                                                                      			  //make sure spot 1 isn't spot 2
			spot2 = rand() % i_pop_size;
		}
		double fit1 = pop.at(spot1).fitness;
		double fit2 = pop.at(spot2).fitness;

		if (fit1 < fit2) {                                                                              			  //first one is better, gets pushed into new vector of policies
			policy A1 = pop.at(spot1);
			ds_pop.push_back(A1);

		}
		if (fit2 <= fit1) {                                                                              			  //second is better
			policy A2 = pop.at(spot2);
			ds_pop.push_back(A2);

		}
	}
	pop = ds_pop;
	assert(pop.size() == i_pop_size / 2);
	return pop;
}


vector<double> mutate(policy& policy, int mut_size, int pop_size) {
	vector<double> weights;
	weights = policy.weights;
	for (int i = 0; i < policy.weights.size(); i++) {
		if (rand() % 2 == 0) {
			weights.at(i) = policy.weights.at(i) + (mut_size * ATRAND) - (mut_size * ATRAND);
		}
	}
	return weights;
}

vector<policy> repopulate(vector<policy> population, int popsize) {
	vector<policy> populationt;
	populationt = population;
	assert(populationt.size() == popsize / 2);
	while (populationt.size() < popsize) {

		int spot = rand() % populationt.size();
		policy A;
		A = populationt.at(spot);
		//cout << "weights size before " << A.weights.size() << endl;
		A.weights = mutate(A, 0.5, popsize);
		//cout << "weights size after " << A.weights.size() << endl;
		populationt.push_back(A);
	}
	assert(populationt.size() == popsize);
	return populationt;
}

double init_dist(ship& boat, goal& goal) {
	double midpointx;
	double midpointy;
	double distance;
	midpointx = (goal.x1 + goal.x2) / 2;
	midpointy = (goal.y1 + goal.y2) / 2;
	distance = sqrt(pow(midpointx - boat.xinit, 2) + pow(midpointy - boat.yinit, 2));
	return distance;
}

vector<policy> simulate(ship& boat, goal& finish, neural_network& NN,
	int pass, int pop_size, int popsize, vector<policy> population, int min, int max, int easytest, int hr4) {
	for (int sim = 0; sim < popsize; sim++) {
		//cout << "policy " << sim << endl;
		//cout << "starting for loop" << endl;
		int tstep = 0;
		int test;
		double init_distance;
		double distance;
		population.at(sim).xpath.push_back(boat.x);
		population.at(sim).ypath.push_back(boat.y);
		distance = init_dist(boat, finish);
		//cout << "initial distance " << distance << endl;
		int n = NN.get_number_of_weights();
		//cout << "weights size" << population.at(sim).weights.size() << endl;
		//cout << "ideal size" << n << endl;
		assert(population.at(sim).weights.size() == n);
		NN.set_weights(population.at(sim).weights, false);
		//cout << "weights set" << endl;
		pass = 0;
		if (easytest == 1) {
			boat.mr1reset();
		}
		while (pass == 0) {

			// MR3
			assert(-pi < boat.theta < pi);
			assert(distance != 0);
			//

			vector<double> vi;
			vi.push_back(distance);
			vi.push_back(boat.theta);
			NN.set_vector_input(vi);
			NN.execute();
			if (easytest == 1) {
				boat.mr1reset();
			}
			//cout << "u before " << boat.u << endl;
			boat.u = NN.get_output(0);
			//cout << "u after = " << boat.u << endl;
			boat.simulate();
			if (easytest == 1) {
				boat.mr1reset();
			}
			population.at(sim).xpath.push_back(boat.x);
			population.at(sim).ypath.push_back(boat.y);

			boat.OOB = shutdown(boat, min, max);
			//cout << "OOB value" << boat.OOB << endl;
			distance = evaluate(boat, finish, population, pop_size, sim);
			population.at(sim).fitness = distance;
			if (boat.OOB == 1) {
				cout << "out of bounds" << endl;
				break;
			}

			if (easytest == 1) {
				//cout << "after" << endl;
				cout << "x " << boat.x << " y " << boat.y << endl;
				//cout << "theta " << boat.theta << " omega " << boat.omega << endl;
			}
			population.at(sim).ts = tstep;
			tstep++;
			if (tstep > 1750) {
				cout << "time expired" << endl;
				break;
			}
			//pass = pass_check(boat, finish);
			pass = passcheckez(boat, finish);

			// MR  4
			if (pass != 0) {
				assert(distance != 0);
			}
			//

			//cout << "distance after one timestep " << distance << endl;
			if (easytest == 1) {
				cin >> test;
			}
		}
		//cout << "fitness " << population.at(sim).fitness << endl;

		//cout << "final x " << boat.x << " final y " << boat.y << endl;

		/*if (boat.OOB == 1) {
		population.at(sim).fitness = population.at(sim).fitness + 1000;
		}

		if (tstep > 2000) {
		population.at(sim).fitness = population.at(sim).fitness + 10000;
		}*/

		if (easytest == 1) {

			// MR1
			double start;
			double end;
			double diff;
			int ypathsize = population.at(sim).ypath.size();
			//cout << "xpath size " << population.at(sim).xpath.size() << endl;
			//cout << "xpath size " << population.at(sim).xpath.size() << endl;
			cout << population.at(sim).ypath.at(0) << endl;
			cout << population.at(sim).ypath.at(ypathsize - 1) << endl;
			start = population.at(sim).ypath.at(0);
			end = population.at(sim).ypath.at(ypathsize - 1);
			//cout << "begin " << start << endl;
			//cout << "end " << end << endl;
			diff = end - start;
			assert(diff < 1.1);
			//
		}
		if (pass == 1) {
			cout << "passed goal" << endl;
			population.at(sim).fitness = population.at(sim).fitness - 1000;
		}
		//reset boat to starting coordinates
		boat.reset();
		if (hr4 == 1) {
			boat.hr4reset();
		}
	}
	return population;
}

int main()
{
	srand(time(NULL));
	///////////////////////////////
	//  		  INITIALIZE  	   //
	///////////////////////////////
	double min = 0;
	double max = 1000;
	ship boat;
	boat.init();
	ship* pboat = &boat;
	goal finish;
	finish.init(max);
	goal* pfinish = &finish;

	int pop_size = 40;

	int pass = 0;
	neural_network NN;

	//cout << "u before " << boat.u << endl;

	//cout << "x and y before " << boat.x << " " << boat.y << endl;

	NN.setup(2, 5, 1);
	NN.set_in_min_max(0, 950 * sqrt(2));
	NN.set_in_min_max(-pi / 0.99, pi / 0.99);
	NN.set_out_min_max(-15.1, 15.1);
	int num_weights;
	num_weights = NN.get_number_of_weights();
	cout << "x start " << boat.xinit << " y start " << boat.yinit << endl;
	cout << "goal x " << finish.x1 << " " << finish.x2 << endl;
	cout << "goal y " << finish.y1 << " " << finish.y2 << endl;
	////////////////////////////////
	//  		  POLICY INIT  		//
	////////////////////////////////
	vector<policy> population;
	for (int i = 0; i < pop_size; i++) {
		policy A;
		A.init(num_weights);
		population.push_back(A);
	}

	int easytest;
	double ez1;
	double ez2;
	cout << "MR1 test? 1/Y, 2/N" << endl;
	cin >> easytest;
	if (easytest == 1) {
		ez1 = finish.x1;
		ez2 = finish.y2;
		boat.mr1test(ez1, ez2);
		cout << "x start " << boat.x << " y start " << boat.y << endl;
		cout << boat.x << " " << boat.y << " " << boat.theta << " " << boat.omega << endl;
	}

	int hr1 = 0;
	int hr3 = 0;
	int hr4 = 0;

	cout << "run HR1 test? 1/Y, 2/N" << endl;
	cin >> hr1;
	if (hr1 == 1) {
		boat.hr1_test();
	}
	cout << "run hr3? 1/Y, 2/N" << endl;
	cin >> hr3;
	// program runs as normal
	cout << "run hr4? 1/Y, 2/N" << endl;
	cin >> hr4;
	// if hr4 != 1, program runs like hr3

	//cout << "population created" << endl;

	////////////////////////////////
	//  		  SIMULATE  		//
	////////////////////////////////
	int max_generations = 20;

	for (int i = 0; i < max_generations; i++) {
		cout << "Generation " << i << endl;
		/*for (int j = 0; j < pop_size; j++) {
		}*/
		//cout << "simulating" << endl;
		//cout << "pop size before simulating" << population.size() << endl;
		population = simulate(boat, finish, NN, pass, pop_size, pop_size, population, min, max, easytest, hr4);
		//cout << "down selecting" << endl;
		//cout << "pop size before downselecting" << population.size() << endl;
		population = down_select(pop_size, population);
		//cout << "repopulating" << endl;
		//cout << "pop size after downselecting" << population.size() << endl;
		population = repopulate(population, pop_size);
		//cout << "pop size after repopulation" << population.size() << endl;
		if (i == 0) {
			ofstream outFile;   																									 // output file
			outFile.open("Ta_Alton_493_ProjectDelta1.csv");   																		 // name of output file
			outFile << 1 << "\n" << endl;
			for (int w = 0; w < 1; w++) {
				for (int q = 0; q < population.at(w).xpath.size() - 1; q++) {
					outFile << population.at(w).xpath.at(q) << ",";
				}

				outFile << endl;

				for (int e = 0; e < population.at(w).ypath.size() - 1; e++) {
					outFile << population.at(w).ypath.at(e) << ",";
				}

				outFile << endl;

				for (int q = 0; q < pop_size; q++) {
					outFile << population.at(q).ts << ",";
				}
			}
			outFile.close();
		}
	}
	ofstream outFile;   																									 // output file
	outFile.open("Ta_Alton_493_ProjectDelta.csv");   																		 // name of output file
	outFile << 1 << "\n" << endl;
	for (int w = 0; w < 1; w++) {
		for (int q = 0; q < population.at(w).xpath.size() - 1; q++) {
			outFile << population.at(w).xpath.at(q) << ",";
		}

		outFile << endl;

		for (int e = 0; e < population.at(w).ypath.size() - 1; e++) {
			outFile << population.at(w).ypath.at(e) << ",";
		}
		
		outFile << endl;

		for (int q = 0; q < pop_size; q++) {
			outFile << population.at(q).ts << ",";
		}
	}
	outFile.close();

	/*ofstream outFile;
	outFile.open("Ta_Alton_493_Delta_LC.csv");
	outFile << 1 << "\n" << endl;
	for (int q = 0; q < pop_size; q++) {
			outFile << population.at(q).ts << ",";
	}
	outFile.close();*/
	return 0;
}
