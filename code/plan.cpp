#include "KinematicChain.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <bits/stdc++.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <functional>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/ValidStateSampler.h>

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;


double computeDistanceToObstacle(const double x, const double y, vector<vector<double>> corners) {
    double minDistance = std::numeric_limits<double>::infinity();
    
    for (int i = 0; i < corners.size(); i++) {
                
        double x_c=corners[i][0];
        double y_c=corners[i][1];

        double distance = sqrt(pow(x - x_c , 2) + pow(y - y_c , 2));
        if (distance < minDistance) {
            minDistance = distance;
        }
    }
    
    return minDistance;
}

class ClearanceObjective : public ompl::base::StateCostIntegralObjective {
public:
    ClearanceObjective(const ompl::base::SpaceInformationPtr &si, const Environment &env)
        : ompl::base::StateCostIntegralObjective(si), obstacles_(env) {}

    ompl::base::Cost stateCost(const ompl::base::State *state) const override {


        auto *compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();

        auto *se2State = compoundState->components[0]->as<ompl::base::SE2StateSpace::StateType>();
        double x=se2State->getX();
        double y=se2State->getY();


        vector<vector<double>> corners;

        for(int i=0;i<obstacles_.size();i++)
        {
            vector<double> xy;
            xy.push_back(obstacles_[i].x0);
            xy.push_back(obstacles_[i].y0);
            corners.push_back(xy);
        }
        
        double clearance = computeDistanceToObstacle(x,y,corners);

        return ompl::base::Cost(1/clearance);
    }


private:
    Environment obstacles_;
};



void writePathToFile(const og::PathGeometric *pathValues,int planningEnvironment)
{

    string fileName=planningEnvironment==1 ? "/code/narrow.txt" : "/code/clear.txt";
    ofstream pathFile(fileName);
       

    if (pathValues)
    {
        // Iterate through each state in the path
        for (std::size_t i = 0; i < pathValues->getStateCount(); ++i)
        {

            double x,y,theta,theta1,theta2,theta3,theta4;

            const ob::State *state = pathValues->getState(i);
            auto *compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();
            // auto *compoundState=ssState;

            auto *se2State = compoundState->components[0]->as<ompl::base::SE2StateSpace::StateType>();
            x=se2State->getX();
            y=se2State->getY();
            theta=se2State->getYaw();

            auto *so2State = compoundState->components[1]->as<ompl::base::SO2StateSpace::StateType>();
            theta1 = so2State->value;

            auto *so2State1 = compoundState->components[2]->as<ompl::base::SO2StateSpace::StateType>();
            theta2 = so2State1->value;

            auto *so2State2 = compoundState->components[3]->as<ompl::base::SO2StateSpace::StateType>();
            theta3 = so2State2->value;

            auto *so2State3 = compoundState->components[4]->as<ompl::base::SO2StateSpace::StateType>();
            theta4 = so2State3->value;

            
            pathFile<<x<<" "<<y<<" "<<theta<<" "<<theta1<<" "<<theta2<<" "<<theta3<<" "<<theta4<<"\n";
        }
    }
    else
    {
        std::cerr << "Failed to cast pathPtr to PathGeometric." << std::endl;
    }

    pathFile.close();

}



void makeScenario1(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{

    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -3;
    start[1] = -3;
    goal[0] = 2 ; 
    goal[1] = 2 ; 
    goal[2] = 0; 
    goal[4] = -0.5*M_PI;

    //Obstacle 1
    env.emplace_back(2, -1, 2.8, -1);
    env.emplace_back(2.8, -1, 2.8, 0.5);
    env.emplace_back(2.8, 0.5, 2, 0.5);
    env.emplace_back(2, 0.5, 2, -1);

    //Obstacle 2
    env.emplace_back(3.2, -1, 4, -1);
    env.emplace_back(4, -1, 4, 0.5);
    env.emplace_back(4, 0.5, 3.2, 0.5);
    env.emplace_back(3.2, 0.5, 3.2, -1);

    std::cout<<"success 1"<<endl;

}

void makeScenario2(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{
    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -4;
    start[1] = -4;
    start[2] = 0;
    goal[0] = 3; 
    goal[1] = 3; 
    goal[2] = 0; 

    //Obstacle 1
    env.emplace_back(-1, -1, 1, -1);
    env.emplace_back(1, -1, 1, 1);
    env.emplace_back(1, 1, -1, 1);
    env.emplace_back(-1, 1, -1, -1);
}

void planScenario1(ompl::geometric::SimpleSetup &ss)
{
    ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));

    ob::PlannerStatus solved = ss.solve(30.0);
    

    if (solved)
    {
        ss.simplifySolution();
        std::cout << "Found solution:" << std::endl;
        ss.getSolutionPath().printAsMatrix(std::cout);
        
        
        og::PathGeometric path=ss.getSolutionPath();

        
        writePathToFile(&path,1);
    }
    else
        std::cout << "No solution found" << std::endl;    

}

// Benchmark Scenario 1 with different PRM sampling strategies (Uniform, Gaussian, Obstacle)
void benchScenario1(og::SimpleSetup &ss) {
    double runtime_limit = 30;   // 30 seconds per run
    double memory_limit = 1024;  // 1024 MB memory limit
    int run_count = 5;           // 5 trials for each sampling strategy

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.1);
    ompl::tools::Benchmark b(ss, "Benchmark_Scenario1");

    // Add PRM with Uniform sampling
    std::cout << "Benchmarking PRM with Uniform Sampling..." << std::endl;
    auto prm_uniform = std::make_shared<og::PRM>(ss.getSpaceInformation());
    prm_uniform->setName("PRM_Uniform");
    b.addPlanner(prm_uniform);

    // Add PRM with Gaussian-based sampling
    std::cout << "Benchmarking PRM with Gaussian Sampling..." << std::endl;
    auto prm_gaussian = std::make_shared<og::PRM>(ss.getSpaceInformation());
    prm_gaussian->setName("PRM_Gaussian");
    prm_gaussian->setProblemDefinition(ss.getProblemDefinition());
    b.addPlanner(prm_gaussian);

    // Add PRM with Obstacle-based sampling
    std::cout << "Benchmarking PRM with Obstacle Sampling..." << std::endl;
    auto prm_obstacle = std::make_shared<og::PRM>(ss.getSpaceInformation());
    prm_obstacle->setName("PRM_Obstacle");
    b.addPlanner(prm_obstacle);

    // Run the benchmark
    std::cout << "Running benchmark for Scenario 1..." << std::endl;
    b.benchmark(request);

    // Save results to log file
    std::cout << "Saving benchmark results to 'benchmark_scenario1.log'..." << std::endl;
    b.saveResultsToFile("benchmark_scenario1.log");
    std::cout << "Benchmark completed. Results saved to 'benchmark_scenario1.log'." << std::endl;
}


void planScenario2(ompl::geometric::SimpleSetup &ss)
{
    ss.setPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));

    ob::PlannerStatus solved = ss.solve(200.0);
    

    if (solved)
    {
        ss.simplifySolution();
        //ss.getSolutionPath().interpolate(20);
        std::cout << "Found solution:" << std::endl;
        ss.getSolutionPath().printAsMatrix(std::cout);
        
        // ob::PathPtr path = ss.getSolutionPath();
        // std::cout << "Found solution:" << std::endl;
        og::PathGeometric path=ss.getSolutionPath();

        // print the path to screen
        //path->print(std::cout);
        writePathToFile(&path,2);
    }
    else
        std::cout << "No solution found" << std::endl;

}

void benchScenario2(og::SimpleSetup &ss) {
    double runtime_limit = 30;   // 30 seconds per run
    double memory_limit = 1024;  // 1024 MB memory limit
    int run_count = 10;          // 10 trials for each planner

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.1);
    ompl::tools::Benchmark b(ss, "Benchmark_Scenario2");

    // Add RRT planner
    std::cout << "Benchmarking RRT..." << std::endl;
    auto rrt = std::make_shared<og::RRT>(ss.getSpaceInformation());
    b.addPlanner(rrt);

    // Add PRM planner
    std::cout << "Benchmarking PRM..." << std::endl;
    auto prm = std::make_shared<og::PRM>(ss.getSpaceInformation());
    b.addPlanner(prm);

    // Add RRTstar planner
    std::cout << "Benchmarking RRTstar..." << std::endl;
    auto rrtstar = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
    b.addPlanner(rrtstar);

    // Run the benchmark
    std::cout << "Running benchmark for Scenario 2..." << std::endl;
    b.benchmark(request);

    // Save results to log file
    std::cout << "Saving benchmark results to 'benchmark_scenario2.log'..." << std::endl;
    b.saveResultsToFile("benchmark_scenario2.log");
    std::cout << "Benchmark completed. Results saved to 'benchmark_scenario2.log'." << std::endl;
}



std::shared_ptr<ompl::base::CompoundStateSpace> createChainBoxSpace()
{   

    auto robot_base_space(std::make_shared<ob::SE2StateSpace>());
    auto robot_joint_1(std::make_shared<ob::SO2StateSpace>());
    auto robot_joint_2(std::make_shared<ob::SO2StateSpace>());
    auto robot_joint_3(std::make_shared<ob::SO2StateSpace>());
    auto robot_joint_4(std::make_shared<ob::SO2StateSpace>());


    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    robot_base_space->setBounds(bounds);

    auto space = std::make_shared<ompl::base::CompoundStateSpace>();  

    space->addSubspace(robot_base_space,1.0);
    space->addSubspace(robot_joint_1,1.0);
    space->addSubspace(robot_joint_2,1.0);
    space->addSubspace(robot_joint_3,1.0);
    space->addSubspace(robot_joint_4,1.0);

    return space;
}


void setupCollisionChecker(ompl::geometric::SimpleSetup &ss, Environment &env)
{   


    ss.setStateValidityChecker([&](const ompl::base::State *state)-> bool{ 

        auto *compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();

        auto *se2State = compoundState->components[0]->as<ompl::base::SE2StateSpace::StateType>();
        double x=se2State->getX();
        double y=se2State->getY();
        double theta=se2State->getYaw();

        // construct a KinematicChainSpace ptr for performing chain collision checking
        auto checker(std::make_shared<KinematicChainSpace>(4,1.0,x,y,theta,&env));

        ompl::base::State *chainState = checker->allocState();
        auto *kinematicState = chainState->as<KinematicChainSpace::StateType>();

        for(int i=0;i<4;i++)
        {
            auto *so2State = compoundState->components[i + 1]->as<ompl::base::SO2StateSpace::StateType>();
            kinematicState->values[i] = so2State->value;
        }


         // construct an instance of  space information from this state space
        auto chain_si(std::make_shared<ob::SpaceInformation>(checker));

        auto collision_checker(std::make_shared<KinematicChainValidityChecker>(chain_si));

        return collision_checker->isValid(state); 
    });




    auto clearanceObjective = std::make_shared<ClearanceObjective>(ss.getSpaceInformation(), env);

    if(env.size()==4)   // implies we are doing planning for scenario 2
    {
        ss.setOptimizationObjective(clearanceObjective);

    }

}

    
int main(int argc, char **argv)
{

    int scenario; 
    Environment env;
    std::vector<double> startVec;
    std::vector<double> goalVec;
    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) Robot Reaching Task" << std::endl;
        std::cout << " (2) Robot Avoiding Task" << std::endl;

        std::cin >> scenario;
    } while (scenario < 1 || scenario > 3);

    switch (scenario)
    {
        case 1:
            makeScenario1(env, startVec, goalVec);
            break;
        case 2:
            makeScenario2(env, startVec, goalVec);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

    auto space = createChainBoxSpace();
    ompl::geometric::SimpleSetup ss(space);

    setupCollisionChecker(ss, env);

    //setup Start and Goal
    ompl::base::ScopedState<> start(space), goal(space);
    space->setup();
    space->copyFromReals(start.get(), startVec);
    space->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);

    switch (scenario)
    {
        case 1:
            planScenario1(ss);
            benchScenario1(ss);
            break;
        case 2:
            planScenario2(ss);
            benchScenario2(ss);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

}




