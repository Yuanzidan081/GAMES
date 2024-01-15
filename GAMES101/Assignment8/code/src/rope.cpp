#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        for (int i = 0; i < num_nodes; ++i)
        {
             // 转成double再做除法，否则就会截断成int
            Vector2D p = start + (end -  start) * (double)i / ((double)num_nodes - 1.0) ;
            masses.emplace_back(new Mass(p, node_mass, false));
        }
        for (int i = 0; i < num_nodes - 1; ++i)
        {
            springs.emplace_back(new Spring(masses[i], masses[i + 1], k));
        }

       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
       }


    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            double dist = (s->m2->position - s->m1->position).norm();
            s->m1->forces += -s->k * (s->m1->position - s->m2->position) / dist * (dist - s->rest_length);
            s->m2->forces += -s->k * (s->m2->position - s->m1->position) / dist * (dist - s->rest_length);
        }
        double damping_factor = 0.005;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                Vector2D acc = (m->forces   - damping_factor * m->velocity)/ m->mass + gravity;

                // 显式方法
                /* m->position += m->velocity * delta_t;
                m->velocity += acc * delta_t; */

                // 隐式方法
                m->velocity += acc * delta_t;
                m->position += m->velocity * delta_t;
                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            double dist = (s->m2->position - s->m1->position).norm();
            s->m1->forces += -s->k * (s->m1->position - s->m2->position) / dist * (dist - s->rest_length);
            s->m2->forces += -s->k * (s->m2->position - s->m1->position) / dist * (dist - s->rest_length);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D acc = m->forces / m->mass + gravity;

                double damping_factor = 0.00005;
                m->position = m->position + (1 - damping_factor) * (m->position - m->last_position) + acc * delta_t * delta_t;
                m->last_position = temp_position;
                // TODO (Part 4): Add global Verlet damping
                
                
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
