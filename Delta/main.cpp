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
	void mr1_test();
	void hr1_test();
	void easytest(double goalx, double goaly);
};

void ship::init() {
	x = (rand() % (200 - 100)) + 100;   				 // boat spawns between 100 and 200 x
	y = (rand() % (600 - 400)) + 400;   				 // boat spawns between 400 and 600 y
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

void ship::reset()  {
	x = xinit;
	y = yinit;
	theta = thetainit;
	omega = omegainit;
	OOB = 0;
}

void ship::xcalc() {
	cout << "v " << v << " theta " << theta << " dt " << dt << endl;
	x = x + (v * sin(theta) * dt);
	cout << "new x " << x << endl;
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

void ship::mr1_test() {
	theta = 0;
	omega = 0;
	thetainit = 0;
	omegainit = 0;
}

void ship::hr1_test() {
	theta = 0;
	thetainit = 0;
}

void ship::easytest(double goalx, double goaly) {
	x = goalx - 1;
	xinit = x;
	y = goaly - 2.5;
	yinit = y;
	theta = 0;
	thetainit = 0;
	omega = 0;
	omegainit = 0;
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
	x1 = rand() % max;
	y1 = 10 + (rand() % max - 10);
	x2 = x1 + 1;
	y2 = y1 - 10;
}

class policy {
public:
	double fitness;
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

int pass_check(ship& agent, goal& goal) {
	double x1;
	double x2;
	double x3;
	double x4;
	double y1;
	double y2;
	double y3;
	double y4;
	double m1;
	double m2;
	double b1;
	double b2;
	double xinter;
	double yinter;
	double check1;
	double check2;
	double epsilon = 0.01;
	double alpha;
	int pass;
	x1 = agent.xprev;
	cout << "x1 = " << x1 << endl;
	x2 = agent.x;
	cout << "x2 = " << x2 << endl;
	x3 = goal.x1;
	cout << "x3 = " << x3 << endl;
	x4 = goal.x2;
	cout << "x4 = " << x4 << endl;
	y1 = agent.yprev;
	cout << "y1 = " << y1 << endl;
	y2 = agent.y;
	cout << "y2 = " << y2 << endl;
	y3 = goal.y1;
	cout << "y3 = " << y3 << endl;
	y4 = goal.y2;
	cout << "y4 = " << y4 << endl;
	m1 = (y2 - y1) / (x2 - x1);
	cout << "m1 " << m1 << endl;
	m2 = (y4 - y3) / (x4 - x3);
	cout << "m2 " << m2 << endl;
	b1 = y1 - (m1 * x1);
	cout << "b1  " << b1 << endl;
	b2 = y3 - (m2 * x3);
	cout << "b2 " << b2 << endl;
	xinter = (b2 - b1) / (m1 - m2);
	cout << "x intercept " << xinter << endl;
	check1 = (m1 * xinter) + b1;
	cout << "check 1 " << check1 << endl;
	check2 = (m2 * xinter) + b2;
	cout << "check 2 " << check2 << endl;
	alpha = fabs(check1 - check2);
	cout << "alpha " << alpha << endl;
	if (alpha < epsilon) {
		pass = 1;
	}
	else {
		pass = 0;
	}
	cout << "pass " << pass << endl;
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
	while (ds_pop.size() / 2 < i_pop_size) {
		int spot1 = rand() % i_pop_size;
		int spot2 = rand() % i_pop_size;
		while (spot2 == spot1) {                                                                           		 //make sure spot 1 isn't spot 2
			spot2 = rand() % i_pop_size;
		}
		double fit1 = pop.at(spot1).fitness;
		double fit2 = pop.at(spot2).fitness;

		if (fit1 < fit2) {                                                                                   		 //first one is better, gets pushed into new vector of policies
			policy A1 = pop.at(spot1);
			ds_pop.push_back(A1);

		}
		if (fit2 <= fit1) {                                                                                   		 //second is better
			policy A2 = pop.at(spot2);
			ds_pop.push_back(A2);

		}
	}
	pop = ds_pop;

	return pop;
}


vector<double> mutate(policy& policy, int mut_size, int pop_size) {
	vector<double> weights;
	for (int i = 0; i < weights.size(); i++) {
		if (rand() % 2 == 0) {
			weights.at(i) = policy.weights.at(i) + (mut_size * ATRAND) - (mut_size * ATRAND);
		}
	}
	return weights;
}

vector<policy> repopulate(vector<policy> population, int popsize) {
	vector<policy> populationt;
	populationt = population;

	while (population.size() < popsize) {
		int spot = rand() % population.size();
		policy A;
		A = population.at(spot);
		A.weights = mutate(A, 0.2, popsize);
		population.push_back(A);
	}
	return population;
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
	int pass, int pop_size, int popsize, vector<policy> population, int min, int max) {
	for (int sim = 0; sim < popsize; sim++) {
		//cout << "starting for loop" << endl;
		int test;
		int tstep = 0;
		double distance;
		population.at(sim).xpath.push_back(boat.x);
		population.at(sim).ypath.push_back(boat.y);
		distance = init_dist(boat, finish);
		//cout << "initial distance " << distance << endl;
		NN.set_weights(population.at(sim).weights, false);
		//cout << "weights set" << endl;
		pass = 0;
		while (pass == 0) {
			vector<double> vi;
			vi.push_back(distance);
			vi.push_back(boat.theta);
			NN.set_vector_input(vi);
			//cout << "input vector set" << endl;
			NN.execute();
			//cout << "NN executed" << endl;
			boat.u = NN.get_output(0);
			//cout << "u = " << boat.u << endl;
			boat.simulate();
			//cout << "one simulation complete" << endl;
			population.at(sim).xpath.push_back(boat.x);
			population.at(sim).ypath.push_back(boat.y);
			//cout << "checking shutdown" << endl;
			boat.OOB = shutdown(boat, min, max);
			//cout << "OOB value" << boat.OOB << endl;
			//cout << "shut down check complete" << endl;
			distance = evaluate(boat, finish, population, pop_size, sim);
			if (boat.OOB == 1) {
				cout << "out of bounds" << endl;
				break;
			}
			//cout << boat.x << " " << boat.y << " " << boat.theta << " " << boat.omega << endl;
			tstep++;
			if (tstep > 750) {
				cout << "time expired" << endl;
				break;
			}
			pass = pass_check(boat, finish);
			//cout << "distance after one timestep " << distance << endl;
			cin >> test;
		}
		population.at(sim).fitness = distance;
		//cout << "final x " << boat.x << " final y " << boat.y << endl;
		if (pass == 1) {
			cout << "passed goal" << endl;
		}
		//reset boat to starting coordinates
		boat.reset();
	}
	return population;
}

int main()
{
	srand(time(NULL));
	///////////////////////////////
	//   		 INITIALIZE   	  //
	///////////////////////////////
	double min = 0;
	double max = 1000;
	ship boat;
	boat.init();
	ship* pboat = &boat;
	goal finish;
	finish.init(max);
	goal* pfinish = &finish;

	int pop_size = 4;

	int pass = 0;
	neural_network NN;

	//cout << "u before " << boat.u << endl;

	//cout << "x and y before " << boat.x << " " << boat.y << endl;

	NN.setup(2, 5, 1);
	NN.set_in_min_max(0, 1000 * sqrt(2));
	NN.set_in_min_max(-pi / 0.99, pi / 0.99);
	NN.set_out_min_max(-15.0 * pi / 180, 15.0 * pi / 180);
	int num_weights;
	num_weights = NN.get_number_of_weights();
	cout << "x start " << boat.xinit << " y start " << boat.yinit << endl;
	cout << "goal x " << finish.x1 << " " << finish.x2 << endl;
	cout << "goal y " << finish.y1 << " " << finish.y2 << endl;
	////////////////////////////////
	//   		 POLICY INIT   	   //
	////////////////////////////////
	vector<policy> population;
	for (int i = 0; i < pop_size; i++) {
		policy A;
		A.init(num_weights);
		population.push_back(A);
	}
	//cout << "population created" << endl;

	int easy;
	cout << "easy test? 1/Y, 2/N" << endl;
	cin >> easy;
	double inp1;
	double inp2;
	inp1 = finish.x1;
	inp2 = finish.y1;
	if (easy == 1) {
		boat.easytest(inp1, inp2);
	}
	cout << "x " << boat.x << endl;
	cout << "y " << boat.y << endl;
	cout << "theta " << boat.theta << endl;
	cout << "omega " << boat.omega << endl;

	////////////////////////////////
	//   		 SIMULATE   	   //
	////////////////////////////////
	int max_generations = 100;

	for (int i = 0; i < max_generations; i++) {
		/*for (int j = 0; j < pop_size; j++) {
		}*/
		//cout << "simulating" << endl;
		population = simulate(boat, finish, NN, pass, pop_size, pop_size, population, min, max);
		//cout << "down selecting" << endl;
		population = down_select(pop_size, population);
		//cout << "repopulating" << endl;
		population = repopulate(population, pop_size);
		cout << "Generation " << i << endl;
	}
	return 0;
}
