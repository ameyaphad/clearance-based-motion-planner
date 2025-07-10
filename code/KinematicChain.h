/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Bryant Gipson, Mark Moll */

#ifndef OMPL_DEMO_KINEMATIC_CHAIN_
#define OMPL_DEMO_KINEMATIC_CHAIN_

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathSimplifier.h>

#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>

// a 2D line segment
struct Segment
{
    Segment(double p0_x, double p0_y, double p1_x, double p1_y) : x0(p0_x), y0(p0_y), x1(p1_x), y1(p1_y)
    {
    }
    double x0, y0, x1, y1;
};

// the robot and environment are modeled both as a vector of segments.
using Environment = std::vector<Segment>;

// simply use a random projection
class KinematicChainProjector : public ompl::base::ProjectionEvaluator
{
public:
    KinematicChainProjector(const ompl::base::StateSpace *space) : ompl::base::ProjectionEvaluator(space)
    {
        int dimension = std::max(2, (int)ceil(log((double)space->getDimension())));
        projectionMatrix_.computeRandom(space->getDimension(), dimension);
    }
    unsigned int getDimension() const override
    {
        return projectionMatrix_.mat.rows();
    }
    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        std::vector<double> v(space_->getDimension());
        space_->copyToReals(v, state);
        projectionMatrix_.project(&v[0], projection);
    }

protected:
    ompl::base::ProjectionMatrix projectionMatrix_;
};

class KinematicChainSpace : public ompl::base::RealVectorStateSpace
{
public:
    KinematicChainSpace(unsigned int numLinks, double linkLength,double x_s, double y_s,double theta, Environment *env = nullptr)
      : ompl::base::RealVectorStateSpace(numLinks), linkLength_(linkLength), environment_(env), x_start_(x_s), y_start_(y_s), theta_(theta)
    {

        ompl::base::RealVectorBounds bounds(numLinks);
        bounds.setLow(-boost::math::constants::pi<double>());
        bounds.setHigh(boost::math::constants::pi<double>());
        setBounds(bounds);
    }

    void registerProjections() override
    {
        registerDefaultProjection(std::make_shared<KinematicChainProjector>(this));
    }

    double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override
    {
        const auto *cstate1 = state1->as<StateType>();
        const auto *cstate2 = state2->as<StateType>();
        double theta1 = 0., theta2 = 0., dx = 0., dy = 0., dist = 0.;

        for (unsigned int i = 0; i < dimension_; ++i)
        {
            theta1 += cstate1->values[i];
            theta2 += cstate2->values[i];
            dx += cos(theta1) - cos(theta2);
            dy += sin(theta1) - sin(theta2);
            dist += sqrt(dx * dx + dy * dy);
        }

        return dist * linkLength_;
    }

    void enforceBounds(ompl::base::State *state) const override
    {
        auto *statet = state->as<StateType>();

        for (unsigned int i = 0; i < dimension_; ++i)
        {
            double v = fmod(statet->values[i], 2.0 * boost::math::constants::pi<double>());
            if (v < -boost::math::constants::pi<double>())
                v += 2.0 * boost::math::constants::pi<double>();
            else if (v >= boost::math::constants::pi<double>())
                v -= 2.0 * boost::math::constants::pi<double>();
            statet->values[i] = v;
        }
    }

    bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override
    {
        bool flag = true;
        const auto *cstate1 = state1->as<StateType>();
        const auto *cstate2 = state2->as<StateType>();

        for (unsigned int i = 0; i < dimension_ && flag; ++i)
            flag &= fabs(cstate1->values[i] - cstate2->values[i]) < std::numeric_limits<double>::epsilon() * 2.0;

        return flag;
    }

    void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t,
                     ompl::base::State *state) const override
    {
        const auto *fromt = from->as<StateType>();
        const auto *tot = to->as<StateType>();
        auto *statet = state->as<StateType>();

        for (unsigned int i = 0; i < dimension_; ++i)
        {
            double diff = tot->values[i] - fromt->values[i];
            if (fabs(diff) <= boost::math::constants::pi<double>())
                statet->values[i] = fromt->values[i] + diff * t;
            else
            {
                if (diff > 0.0)
                    diff = 2.0 * boost::math::constants::pi<double>() - diff;
                else
                    diff = -2.0 * boost::math::constants::pi<double>() - diff;

                statet->values[i] = fromt->values[i] - diff * t;
                if (statet->values[i] > boost::math::constants::pi<double>())
                    statet->values[i] -= 2.0 * boost::math::constants::pi<double>();
                else if (statet->values[i] < -boost::math::constants::pi<double>())
                    statet->values[i] += 2.0 * boost::math::constants::pi<double>();
            }
        }
    }

    double linkLength() const
    {
        return linkLength_;
    }

    const Environment *environment() const
    {
        return environment_;
    }

    double x_start() const
    {
        return x_start_;
    }

    double y_start() const
    {
        return y_start_;
    }

    double inclination() const
    {
        return theta_;
    }

protected:
    double linkLength_;
    Environment *environment_;

    // denotes start coordinates of the link
    double x_start_;
    double y_start_;

    // angle for square base of robot
    double theta_;
};

class KinematicChainValidityChecker : public ompl::base::StateValidityChecker
{
public:
    KinematicChainValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
    }

    bool isValid(const ompl::base::State *state) const override
    {
        const KinematicChainSpace *space = si_->getStateSpace()->as<KinematicChainSpace>();
        const auto *s = state->as<KinematicChainSpace::StateType>();

        return isValidImpl(space, s);
    }

protected:
    bool isValidImpl(const KinematicChainSpace *space, const KinematicChainSpace::StateType *s) const
    {
        unsigned int n = si_->getStateDimension();
        Environment segments;
        double linkLength = space->linkLength();
        double theta = 0., x = space->x_start(), y = space->y_start(), xN, yN;

        segments.reserve(n + 1);
        for (unsigned int i = 0; i < n; ++i)
        {
            theta += s->values[i];
            xN = x + cos(theta) * linkLength;
            yN = y + sin(theta) * linkLength;
            segments.emplace_back(x, y, xN, yN);
            x = xN;
            y = yN;
        }
        xN = x + cos(theta) * 0.001;
        yN = y + sin(theta) * 0.001;
        segments.emplace_back(x, y, xN, yN);


        // construct square corners and convert to segments
        Environment squareSegments;

        double x1,y1,x2,y2,side_length=1.0;

        x1=(space->x_start() + (cos(space->inclination())*side_length/2.0) - (sin(space->inclination())*side_length/2.0));
        y1=(space->y_start() + (cos(space->inclination())*side_length/2.0) + (sin(space->inclination())*side_length/2.0));

        x2=(space->x_start() + (cos(space->inclination())*-1.0*side_length/2.0) - (sin(space->inclination())*side_length/2.0));
        y2=(space->y_start() + (cos(space->inclination())*-1.0*side_length/2.0) + (sin(space->inclination())*side_length/2.0));

        squareSegments.emplace_back(x1,y1,x2,y2);

        x1=(space->x_start() + (cos(space->inclination())*-1.0*side_length/2.0) - (sin(space->inclination())*-1.0*side_length/2.0));
        y1=(space->y_start() + (cos(space->inclination())*-1.0*side_length/2.0) + (sin(space->inclination())*-1.0*side_length/2.0));

        squareSegments.emplace_back(x1,y1,x2,y2);

        x2=(space->x_start() + (cos(space->inclination())*side_length/2.0) - (sin(space->inclination())*-1.0*side_length/2.0));
        y2=(space->y_start() + (cos(space->inclination())*side_length/2.0) + (sin(space->inclination())*-1.0*side_length/2.0));

        squareSegments.emplace_back(x1,y1,x2,y2);

        x1=(space->x_start() + (cos(space->inclination())*side_length/2.0) - (sin(space->inclination())*side_length/2.0));
        y1=(space->y_start() + (cos(space->inclination())*side_length/2.0) + (sin(space->inclination())*side_length/2.0));

        squareSegments.emplace_back(x1,y1,x2,y2);




        return selfIntersectionTest(segments) && environmentIntersectionTest(segments, *space->environment()) && squareIntersectionTest(squareSegments,*space->environment());
    }

    // return true iff env does *not* include a pair of intersecting segments
    bool selfIntersectionTest(const Environment &env) const
    {
        for (unsigned int i = 0; i < env.size(); ++i)
            for (unsigned int j = i + 1; j < env.size(); ++j)
                if (intersectionTest(env[i], env[j]))
                    return false;
        return true;
    }
    // return true iff no segment in env0 intersects any segment in env1
    bool environmentIntersectionTest(const Environment &env0, const Environment &env1) const
    {
        for (const auto &i : env0)
            for (const auto &j : env1)
                if (intersectionTest(i, j))
                    return false;
        return true;
    }


    bool squareIntersectionTest(const Environment &env0, const Environment &env1) const
    {
        for (const auto &i : env0)
            for (const auto &j : env1)
                if (intersectionTest(i, j))
                    return false;
        return true;
    }

    // return true iff segment s0 intersects segment s1
    bool intersectionTest(const Segment &s0, const Segment &s1) const
    {
        // adopted from:
        // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/1201356#1201356
        double s10_x = s0.x1 - s0.x0;
        double s10_y = s0.y1 - s0.y0;
        double s32_x = s1.x1 - s1.x0;
        double s32_y = s1.y1 - s1.y0;
        double denom = s10_x * s32_y - s32_x * s10_y;
        if (fabs(denom) < std::numeric_limits<double>::epsilon())
            return false;  // Collinear
        bool denomPositive = denom > 0;

        double s02_x = s0.x0 - s1.x0;
        double s02_y = s0.y0 - s1.y0;
        double s_numer = s10_x * s02_y - s10_y * s02_x;
        if ((s_numer < std::numeric_limits<float>::epsilon()) == denomPositive)
            return false;  // No collision
        double t_numer = s32_x * s02_y - s32_y * s02_x;
        if ((t_numer < std::numeric_limits<float>::epsilon()) == denomPositive)
            return false;  // No collision
        if (((s_numer - denom > -std::numeric_limits<float>::epsilon()) == denomPositive) ||
            ((t_numer - denom > std::numeric_limits<float>::epsilon()) == denomPositive))
            return false;  // No collision
        return true;
    }
};



Environment createHornEnvironment(unsigned int d, double eps)
{
    std::ofstream envFile(boost::str(boost::format("environment_%i.dat") % d));
    std::vector<Segment> env;
    double w = 1. / (double)d, x = w, y = -eps, xN, yN, theta = 0.,
           scale = w * (1. + boost::math::constants::pi<double>() * eps);

    envFile << x << " " << y << std::endl;
    for (unsigned int i = 0; i < d - 1; ++i)
    {
        theta += boost::math::constants::pi<double>() / (double)d;
        xN = x + cos(theta) * scale;
        yN = y + sin(theta) * scale;
        env.emplace_back(x, y, xN, yN);
        x = xN;
        y = yN;
        envFile << x << " " << y << std::endl;
    }

    theta = 0.;
    x = w;
    y = eps;
    envFile << x << " " << y << std::endl;
    scale = w * (1.0 - boost::math::constants::pi<double>() * eps);
    for (unsigned int i = 0; i < d - 1; ++i)
    {
        theta += boost::math::constants::pi<double>() / d;
        xN = x + cos(theta) * scale;
        yN = y + sin(theta) * scale;
        env.emplace_back(x, y, xN, yN);
        x = xN;
        y = yN;
        envFile << x << " " << y << std::endl;
    }
    envFile.close();
    return env;
}

Environment createEmptyEnvironment(unsigned int d)
{
    std::ofstream envFile(boost::str(boost::format("environment_%i.dat") % d));
    std::vector<Segment> env;
    envFile.close();
    return env;
}

#endif
