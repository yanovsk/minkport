//
//  MyEigenWrapper.mm
//  eigentest
//
//  Created by Dima Yanovsky on 3/30/25.
//

// MyEigenWrapper.mm
#import "MyEigenWrapper.h"
#include <Eigen/Core>

NSString* eigenVersionString(void) {
    // Eigen version is available via macros.
    return [NSString stringWithFormat:@"Eigen Version %d.%d.%d",
            EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION];
}
