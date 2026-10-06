#include "diagnostics.h"
#include <cassert>
#include <iostream>
int main(){
 auto& logger=fastlivo_diag::Logger::instance();assert(logger.enabled());
 assert(!logger.detail_at(115.-1e-9));assert(logger.detail_at(115.));
 assert(logger.detail_at(116.));assert(logger.detail_at(118.));assert(!logger.detail_at(118.+1e-9));
 logger.set_context(1,80000000000ULL);assert(!logger.detail());assert(logger.detail_at(116.));
 logger.set_context(1,116000000000ULL);assert(logger.detail());assert(!logger.detail_at(80.));
 logger.set_context(1,120000000000ULL);assert(!logger.detail());
 std::cout<<"PASS: inclusive original predicate,3s window,raw source time independent of estimator context; no timestamps rewritten\n";
}
