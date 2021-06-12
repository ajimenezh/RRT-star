# RRT* (Rapidly-exploring Random Tree)

Rapidly-exploring Random Tree (RRT) is a sampling-based algorithm for motion planning. The RRT* is an improved method to ensure asymptotic optimality.

## Introduction

Motion planning is the problem to find
a sequence of inputs that drives the system from its initial
condition to the goal region, while avoiding collision with
obstacles, given a description of
its dynamics, an initial state, a final state, a set of obstacles,
and a goal region. 

RRT is a sampling-based method which randomly sample a set of states from the state-space.

## Problem

$$ \dot{x}(t) = f(x(t), u(t))$$

$$ x(0) = x_0$$



## Find the Tangent to Two Circles

The line tangent to two circles can be found algebraically with the condition that the distance of the line to the center of each circle is equal to the radius of the circle, and solving the system of equations. To simplify the equations, we can assume that one of the circles is centered at (0, 0), and then we can translate the solution. If we use the representation of the line as $ax + by + c = 0$, the distance to a point ($p_x$, $p_y$) is $|a p_x + b p_y + c|$. So the equations are:

$$ |a \cdot 0 + b \cdot 0 + c| = r_1$$

$$ |a \cdot p_x + b \cdot p_y + c| = r_2$$

And we can use the following equation of the line so the answer is unique:

$$ a^2 + b^2 = 1$$

This system has up to 4 solutions because the absolute value can contain both a positive and negative value.

$$ a \cdot 0 + b \cdot 0 + c = \pm r_1$$

$$ a \cdot p_x + b \cdot p_y + c = \pm r_2$$

So if we define $s_i = \pm r_i$ then:

$$ c = s_i$$

$$ a \cdot p_x + b \cdot p_y = s_2 - s_1$$

$$ (a \cdot p_x)^2 + (s_2 - s_1)^2 - 2*(a \cdot p_x)(s_2 - s_1) = (1 - a^2)p_y^2$$

$$ a^2 (p_x^2 + p_y^2) - 2*a(s_2 - s_1)p_x + (s_2 - s_1)^2 - p_y^2 = 0$$

We can solve this:

$$ a = \frac{-2(s_2 - s_1)p_x \pm \sqrt{ 4(s_2 - s_1)^2p_x^2 - 4(p_x^2 + p_y^2)((s_2 - s_1)^2 - p_y^2) }}{2(p_x^2 + p_y^2)} $$

$$ a = \frac{(s_2 - s_1)p_x \pm p_y \sqrt{ (p_x^2 + p_y^2) - (s_2 - s_1)^2}}{p_x^2 + p_y^2} $$

And then b with:

$$ b = \pm \sqrt{1 - a^2} $$

Since we have already taken both + and - above it doesn't matter which sign we take in these equations as long as we are consistent and take always the same sign.

```
namespace {
	const double EPS = 1.0e-6;


	std::optional<Line> CalcTangentLine(const Point& p, double s1, double s2) {
		double z = p.x * p.x + p.y * p.y;
		double delta = z - (s2 - s1)* (s2 - s1);
		
		if (delta < -EPS) {
			return std::nullopt;
		}

		double a = (p.x * (s2 - s1) + p.y * sqrt(delta)) / (z);
		double b = (p.y * (s2 - s1) - p.x * sqrt(delta)) / (z);
		double c = s1;

		return Line(a, b, c);
	}

}

std::vector<Line> FindTangentLines(const Circle& c1, const Circle& c2) {
	double r1 = c1.Radius();
	double r2 = c2.Radius();

	std::vector<Line> result;

	for (int i = -1; i <= 1; i += 2) {
		double s1 = i * r1;
		for (int j = -1; j <= 1; j += 2) {
			double s2 = j * r2;
			std::optional<Line> res = CalcTangentLine(c2.Center() - c1.Center(), s1, s2);

			if (res.has_value()) {
				Line line = res.value();
				line.c -= line.a* c1.Center().x + line.b * c1.Center().y;
				result.push_back(line);
			}
		}
	}

	return result;
}
```

