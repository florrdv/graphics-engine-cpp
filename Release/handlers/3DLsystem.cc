#include "3DLsystem.h"

#include <cmath>
#include <stack>
#include <math.h>

#include "Universal.h"

struct Vector3DQuadruplet {
    Vector3D first, second, third, fourth;
};

void draw3DLSystem(const LParser::LSystem3D& l_system, Figure& figure, const Color color, std::string current, int it) {
    int iterations = l_system.get_nr_iterations();
    std::string initiator = l_system.get_initiator();
    std::set<char> alphabet = l_system.get_alphabet();

    if (current == "") current = initiator;

    if (it == iterations) {
        Vector3D cur = Vector3D::point(0, 0, 0);

        Vector3D h = Vector3D::point(1, 0, 0);
        Vector3D l = Vector3D::point(0, 1, 0);
        Vector3D u = Vector3D::point(0, 0, 1);

        double angleOffset = l_system.get_angle() * M_PI / 180;

        std::stack<Vector3DQuadruplet> stack;

        for (char c : current) {
            if (c == '+') {
                Vector3D newH = h * std::cos(angleOffset) + l * std::sin(angleOffset);
                Vector3D newL = -h * std::sin(angleOffset) + l * std::cos(angleOffset);

                h = newH;
                l = newL;
            }
            else if (c == '-') {
                Vector3D newH = h * std::cos(-angleOffset) + l * std::sin(-angleOffset);
                Vector3D newL = -h * std::sin(-angleOffset) + l * std::cos(-angleOffset);

                h = newH;
                l = newL;
            }
            else if (c == '^') {
                Vector3D newH = h * std::cos(angleOffset) + u * std::sin(angleOffset);
                Vector3D newU = -h * std::sin(angleOffset) + u * std::cos(angleOffset);

                h = newH;
                u = newU;
            }
            else if (c == '&') {
                Vector3D newH = h * std::cos(-angleOffset) + u * std::sin(-angleOffset);
                Vector3D newU = -h * std::sin(-angleOffset) + u * std::cos(-angleOffset);

                h = newH;
                u = newU;
            }
            else if (c == '\\') {
                Vector3D newL = l * std::cos(angleOffset) - u * std::sin(angleOffset);
                Vector3D newU = l * std::sin(angleOffset) + u * std::cos(angleOffset);

                l = newL;
                u = newU;
            }
            else if (c == '/') {
                Vector3D newL = l * std::cos(-angleOffset) - u * std::sin(-angleOffset);
                Vector3D newU = l * std::sin(-angleOffset) + u * std::cos(-angleOffset);

                l = newL;
                u = newU;
            }
            else if (c == '|') {
                h = -h;
                l = -l;
            }
            else if (c == '(') stack.push(Vector3DQuadruplet{ cur, h, l, u });
            else if (c == ')') {
                Vector3DQuadruplet saved = stack.top();
                stack.pop();

                cur = saved.first;
                h = saved.second;
                l = saved.third;
                u = saved.fourth;
            }
            else if (alphabet.find(c) != alphabet.end()) {
                if (l_system.draw(c)) {
                    Vector3D p1 = cur;
                    cur += h;
                    Vector3D p2 = cur;

                    figure.points.push_back(p1);
                    int i = figure.points.size() - 1;
                    figure.points.push_back(p2);

                    figure.faces.push_back(Face({ i, i + 1 }));
                }
                else cur += h;
            }
            else std::cout << ("⛔️| Invalid character " + std::to_string(c) + " in l-system description") << std::endl;
        }

        return;
    }

    std::string replaced = "";
    for (char c : current) {
        if (alphabet.find(c) == alphabet.end()) replaced += c;
        else {
            replaced += l_system.get_replacement(c);
        }
    }

    draw3DLSystem(l_system, figure, color, replaced, it + 1);
}