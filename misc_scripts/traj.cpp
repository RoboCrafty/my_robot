#include <iostream>
#include <fstream>




struct cubic_coeff{
    double a0{0}, a1{0}, a2{0}, a3{0};
};

struct quintic_coeff{
    double a0{0}, a1{0}, a2{0}, a3{0}, a4{0};
    double a5{0};
};

struct xy_container{
    double x;
    double t;
};

cubic_coeff calcCubicPoly(double q_initial, double q_final, double t)
{
    cubic_coeff result;
    double s, sdot, sddot, a0, a1, a2, a3;

    double dx = q_final - q_initial;
    result.a0 = q_initial;
    result.a1 = 0;
    result.a2 = (3 * dx) / (t * t);
    result.a3 = (-2 * dx) / (t*t*t);
    return result;
}

quintic_coeff calcQuinticPoly(double q_initial, double q_final, double T){

    quintic_coeff result;

    double dx = q_final - q_initial;
    result.a0 = q_initial;
    std::cout << "q_initial give is: " << q_initial;
    result.a1 = 0;
    result.a2 = 0;

    result.a5 = (6*dx)/(T*T*T*T*T);
    result.a4 = (-15*dx)/(T*T*T*T);
    result.a3 = (10*dx)/(T*T*T);
    return result;
}


xy_container eval_q(cubic_coeff coeff, double t)
{
    xy_container result;
    result.x = coeff.a0 + coeff.a1 * t + coeff.a2 * t * t + coeff.a3 * t * t * t;
    result.t = t;
    return result;
}
xy_container eval_qdot(cubic_coeff coeff, double t)
{
    xy_container result;
    result.x = coeff.a1 + 2 * coeff.a2 * t + 3 * coeff.a3 * t * t;
    result.t = t;
    return result;
}
xy_container eval_qddot(cubic_coeff coeff, double t)
{
    xy_container result;
    result.x = 2 * coeff.a2 + 6 * coeff.a3 * t;
    result.t = t;
    return result;
}

// xy_container eval_q(quintic_coeff coeff, double t)
// {
//     xy_container result;
//     result.x = coeff.a0 + coeff.a1 * t + coeff.a2 * t * t + coeff.a3 * t * t * t + coeff.a4*t*t*t*t + coeff.a5*t*t*t*t*t;
//     result.t = t;
//     return result;
// }
// xy_container eval_qdot(quintic_coeff coeff, double t)
// {
//     xy_container result;
//     result.x = coeff.a1 + 2 * coeff.a2 * t + 3 * coeff.a3 * t * t + 4*coeff.a4*t*t*t + 5*coeff.a5*t*t*t*t;
//     result.t = t;
//     return result;
// }
// xy_container eval_qddot(quintic_coeff coeff, double t)
// {
//     xy_container result;
//     result.x = 2 * coeff.a2 + 6 * coeff.a3 * t + 12*coeff.a4*t*t + 20*coeff.a5*t*t*t;
//     result.t = t;
//     return result;
// }




int main()
{
    double q_initial = 2;
    double q_final = 6;
    double T = ;

    double dt = 0.01;


    double t = 0;
    auto coeff = calcCubicPoly(q_initial, q_final, T);
    // std::cout << "coeffs are --- a0:" << coeff.a0 << " a1:" << coeff.a1 << " a2:" << coeff.a2 << " a3:" << coeff.a3 << " a4: " << coeff.a4 << " a5: " << coeff.a5 << std::endl;
    
    std::ofstream file("joint_log.csv");
    file << "t,q,qdot,qddot\n";

    for(double i = 0; i<=T; i += dt)
    {
        auto res = eval_q(coeff, i);
        auto res2 = eval_qdot(coeff, i);
        auto res3 = eval_qddot(coeff, i);
        file << res.t << "," << res.x << "," << res2.x << "," << res3.x << "\n";
        // t += dt;
        std::cout << res.x << "," << res.t << std::endl;
    }


}



