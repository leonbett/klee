//===-- TimingSolver.cpp --------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "TimingSolver.h"

#include "klee/Config/Version.h"
#include "klee/ExecutionState.h"
#include "klee/Solver.h"
#include "klee/Statistics.h"
#include "SeedInfo.h"
#include "klee/TimerStatIncrementer.h"

#include "CoreStats.h"

using namespace klee;
using namespace llvm;


ref<Expr> TimingSolver::ZESTEvaluate(const ExecutionState& state, ref<Expr> expr) {
  std::map<ExecutionState*, std::vector<SeedInfo> >::iterator its =
    seedMap->find(const_cast<ExecutionState*>(&state));

  //Paul need to understand better when this can happen
  if (its != seedMap->end()) {
    //Paul: need to understand better how the size of the seed vector varies
    assert(its->second.size() <= 1);
    std::vector<SeedInfo>::iterator siit = its->second.begin();

    if (siit != its->second.end()) { // Leon: SeedInfo was found
      return siit->assignment.evaluate(expr); // Leon: Evaluate expr on seedinfo assignment
    }
  }
  return expr;
}



bool TimingSolver::evaluate(const ExecutionState& state, ref<Expr> expr,
                            Solver::Validity &result, bool useSeeds) {
  // Fast path, to avoid timer and OS overhead.
  if (seedMap && useSeeds) { // Leon: useSeeds -> concrete
    if (ConstantExpr *CE = dyn_cast<ConstantExpr>(expr)) { // Leon: expr is constant
      result = CE->isTrue() ? Solver::True : Solver::False;
      return true;
    }
    expr = ZESTEvaluate(state, expr); // Leon: expr is not constant -> ZESTEvaluate
  }

  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(expr)) {
    result = CE->isTrue() ? Solver::True : Solver::False;
    return true;
  }

  TimerStatIncrementer timer(stats::solverTime);

  if (simplifyExprs)
    expr = state.constraints.simplifyExpr(expr);

  bool success = solver->evaluate(Query(state.constraints, expr), result);

  state.queryCost += timer.check();

  return success;
}

bool TimingSolver::mustBeTrue(const ExecutionState& state, ref<Expr> expr, 
                              bool &result, bool useSeeds) {
  // Fast path, to avoid timer and OS overhead.
  if (seedMap && useSeeds) {
    if (ConstantExpr *CE = dyn_cast<ConstantExpr>(expr)) {
      result = CE->isTrue() ? true : false;
      return true;
    }
    expr = ZESTEvaluate(state, expr);
  }

  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(expr)) {
    result = CE->isTrue() ? true : false;
    return true;
  }

  errs() << "mustBeTrue: beyond zestevaluate, doing real solving\n";


  TimerStatIncrementer timer(stats::solverTime);

  if (simplifyExprs)
    expr = state.constraints.simplifyExpr(expr);

  bool success = solver->mustBeTrue(Query(state.constraints, expr), result);

  state.queryCost += timer.check();

  return success;
}

bool TimingSolver::mustBeFalse(const ExecutionState& state, ref<Expr> expr,
                               bool &result, bool useSeeds) {
  return mustBeTrue(state, Expr::createIsZero(expr), result, useSeeds);
}

bool TimingSolver::mayBeTrue(const ExecutionState& state, ref<Expr> expr, 
                             bool &result, bool useSeeds) {
  bool res;
  if (!mustBeFalse(state, expr, res, useSeeds))
    return false;
  result = !res;
  return true;
}

bool TimingSolver::mayBeFalse(const ExecutionState& state, ref<Expr> expr, 
                              bool &result, bool useSeeds) {
  bool res;
  if (!mustBeTrue(state, expr, res, useSeeds))
    return false;
  result = !res;
  return true;
}

bool TimingSolver::getValue(const ExecutionState& state, ref<Expr> expr, 
                            ref<ConstantExpr> &result, bool useSeeds) {
  // Fast path, to avoid timer and OS overhead.
  if (seedMap && useSeeds) {
    expr = ZESTEvaluate(state, expr);
  }
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(expr)) {
    result = CE;
    return true;
  }

  errs() << "getValue: beyond zestevaluate, doing real solving\n";

  
  TimerStatIncrementer timer(stats::solverTime);

  if (simplifyExprs)
    expr = state.constraints.simplifyExpr(expr);

  bool success = solver->getValue(Query(state.constraints, expr), result);

  state.queryCost += timer.check();

  return success;
}

bool 
TimingSolver::getInitialValues(const ExecutionState& state, 
                               const std::vector<const Array*>
                                 &objects,
                               std::vector< std::vector<unsigned char> >
                                 &result,
                                bool useSeeds) {
  if (objects.empty())
    return true;

  TimerStatIncrementer timer(stats::solverTime);

  bool success = solver->getInitialValues(Query(state.constraints,
                                                ConstantExpr::alloc(0, Expr::Bool)), 
                                          objects, result);

  state.queryCost += timer.check();
  
  return success;
}

std::pair< ref<Expr>, ref<Expr> >
TimingSolver::getRange(const ExecutionState& state, ref<Expr> expr, int useSeeds) {
  if (seedMap && useSeeds) {
    expr = ZESTEvaluate(state, expr);
  }

  return solver->getRange(Query(state.constraints, expr));
}