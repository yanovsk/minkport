#import "MyEigenWrapper.h"

// Include public Eigen and qpsolvers-eigen headers.
#include <QpSolversEigen/QpSolversEigen.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>

NSArray* solveExampleQP(void) {
    // Define a minimal QP:
    //   minimize 0.5 * x^T * H * x + g^T * x
    //   subject to: A * x within given bounds.
    
    // Hessian: 2x2 sparse matrix.
    Eigen::SparseMatrix<double> H(2, 2);
    H.insert(0, 0) = 4;
    H.insert(0, 1) = 1;
    H.insert(1, 0) = 1;
    H.insert(1, 1) = 2;
    
    // Constraint matrix: 3x2 sparse matrix.
    Eigen::SparseMatrix<double> A(3, 2);
    A.insert(0, 0) = 1;  A.insert(0, 1) = 1;
    A.insert(1, 0) = 1;
    A.insert(2, 1) = 1;
    
    // Gradient vector (2x1).
    Eigen::Matrix<double, 2, 1> grad;
    grad << 1, 1;
    
    // Lower and upper bounds for the constraints (3x1).
    Eigen::Matrix<double, 3, 1> lb, ub;
    lb << 1, 0, 0;
    ub << 1, 0.7, 0.7;
    
    // Instantiate and configure the solver.
    QpSolversEigen::Solver solver;
    // Use the proxqp backend.
    if (!solver.instantiateSolver("proxqp")) {
        std::cerr << "Error instantiating proxqp solver" << std::endl;
        return nil;
    }
    
    // Set proxqp-specific parameters if needed.
    if (solver.getSolverName() == "proxqp") {
        solver.setBooleanParameter("verbose", true);
        // Set a proxqp-specific parameter: initial guess strategy, for example.
        solver.setStringParameter("initial_guess", "WARM_START");
        // You can set other proxqp parameters here if desired.
    }
    
    // Set up problem dimensions and data.
    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfConstraints(3);
    solver.data()->setHessianMatrix(H);
    solver.data()->setGradient(grad);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(lb);
    solver.data()->setUpperBound(ub);
    
    if (!solver.initSolver() ||
        solver.solveProblem() != QpSolversEigen::ErrorExitFlag::NoError) {
        std::cerr << "Error solving the QP" << std::endl;
        return nil;
    }
    
    // Retrieve the solution (Eigen 2x1 vector).
    Eigen::Matrix<double, 2, 1> sol = solver.getSolution();
    
    // Convert the Eigen solution vector to an NSArray for Swift.
    NSMutableArray *resultArray = [NSMutableArray arrayWithCapacity:2];
    for (int i = 0; i < sol.rows(); i++) {
        [resultArray addObject:@(sol(i, 0))];
    }
    
    return resultArray;
}
