//===-- AflIdAssignPass.cpp -------------------------------------------------------===//

#include "Passes.h"

#include "klee/Config/Version.h"

#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/CallSite.h"
#include "llvm/Support/raw_ostream.h"

namespace klee {

char AflIdAssignPass::ID;

//savior
static inline bool has_bbid_instrumentation(llvm::BasicBlock &bb){
  for (llvm::Instruction& inst : bb.getInstList()) {
    if (inst.getMetadata("afl_cur_loc") != NULL) {
      return true;
    }
  }
  return false;
}


// This pass assigns some BBID to BBs that do not have one, because they were transformed by KLEE.
bool AflIdAssignPass::runOnModule(llvm::Module &M) {
  llvm::LLVMContext &C = M.getContext();
  llvm::IntegerType *Int32Ty = llvm::IntegerType::getInt32Ty(C);
  unsigned int n = 0;

  for (auto &F : M) {
    for (auto &BB : F) {
      if (!has_bbid_instrumentation(BB)) {
        for (llvm::Instruction& instr : BB.getInstList()) {
            // Label the first instruction
            unsigned int loc = rand();
            //llvm::errs() << "AflIdAssignPass loc: " << loc << "\n";
            llvm::ConstantInt *CurLoc = llvm::ConstantInt::get(Int32Ty, loc);
            auto meta_loc = llvm::MDNode::get(C, llvm::ConstantAsMetadata::get(CurLoc));
            instr.setMetadata("afl_cur_loc", meta_loc);
            n++;
            break;
        }
      }
    }
  }

  llvm::errs() << "AflIdAssignPass instrumented " << n << " BBs.\n";
  return true;
}

}

    
