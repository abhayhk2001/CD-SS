// Clang includes
#include <clang/AST/ASTConsumer.h>
#include <clang/AST/ASTContext.h>
#include <clang/AST/Expr.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>
#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/Analysis/CFG.h>
#include <clang/Basic/Diagnostic.h>
#include <clang/Basic/LangOptions.h>
#include <clang/Frontend/CompilerInstance.h>
#include <clang/Frontend/FrontendAction.h>
#include <clang/Tooling/CommonOptionsParser.h>
#include <clang/Tooling/Tooling.h>

#include "clang/AST/RecursiveASTVisitor.h"


// LLVM includes
#include <llvm/ADT/StringRef.h>
#include <llvm/Support/Casting.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>

namespace McCabe {
class LHSVariableFinder final
: public clang::RecursiveASTVisitor<LHSVariableFinder> {
 public:
  static bool
  find(clang::FunctionDecl* CandidateFunction, std::string var_name) {
    LHSVariableFinder ActualFinder;
    ActualFinder.var = var_name;
    ActualFinder.TraverseDecl(CandidateFunction);
    return ActualFinder.Found;
  }


  bool VisitBinaryOperator(clang::BinaryOperator* SymbolUse) {
    if (auto* LHSexpr = SymbolUse->getLHS()) {
      std::string usedVar;
      if (clang::DeclRefExpr* DRE =
              clang::dyn_cast<clang::DeclRefExpr>(LHSexpr)) {
        if (clang::VarDecl* VD =
                clang::dyn_cast<clang::VarDecl>(DRE->getDecl())) {
          usedVar = VD->getQualifiedNameAsString();
        }
      }

      else if (clang::MemberExpr* ME =
                   clang::dyn_cast<clang::MemberExpr>(LHSexpr)) {
        if (clang::DeclRefExpr* DRE =
                clang::dyn_cast<clang::DeclRefExpr>(ME->getBase())) {
          if (clang::VarDecl* VD =
                  clang::dyn_cast<clang::VarDecl>(DRE->getDecl())) {
            usedVar = VD->getQualifiedNameAsString();
          }
        }
      }

      else if (clang::CXXOperatorCallExpr* OCE =
                   clang::dyn_cast<clang::CXXOperatorCallExpr>(LHSexpr)) {
        if (clang::DeclRefExpr* DRE =
                clang::dyn_cast<clang::DeclRefExpr>(OCE->getArg(0))) {
          if (clang::VarDecl* VD =
                  clang::dyn_cast<clang::VarDecl>(DRE->getDecl())) {
            usedVar = VD->getQualifiedNameAsString();
          }
        }
      }

      else {
      }
      // llvm::outs() << "usedVar " << usedVar << " var "<< var <<"\n";

      if (usedVar == var) {
        Found = true;
        return false;
      }
    }
    return true;
  }


  bool VisitUnaryOperator(clang::UnaryOperator* SymbolUse) {
    if (auto* LHSexpr = SymbolUse->getSubExpr()) {
      std::string usedVar;
      if (clang::DeclRefExpr* DRE =
              clang::dyn_cast<clang::DeclRefExpr>(LHSexpr)) {
        if (clang::VarDecl* VD =
                clang::dyn_cast<clang::VarDecl>(DRE->getDecl())) {
          usedVar = VD->getQualifiedNameAsString();
        }
      }

      else if (clang::MemberExpr* ME =
                   clang::dyn_cast<clang::MemberExpr>(LHSexpr)) {
        if (clang::DeclRefExpr* DRE =
                clang::dyn_cast<clang::DeclRefExpr>(ME->getBase())) {
          if (clang::VarDecl* VD =
                  clang::dyn_cast<clang::VarDecl>(DRE->getDecl())) {
            usedVar = VD->getQualifiedNameAsString();
          }
        }
      }

      else {
      }

      // llvm::outs() << "usedVar " << usedVar << " var "<< var <<"\n";

      if (usedVar == var) {
        Found = true;
        return false;
      }
    }
    return true;
  }

  bool VisitCXXOperatorCallExpr(clang::CXXOperatorCallExpr* SymbolUse) {
    if (clang::DeclRefExpr* DRE =
            clang::dyn_cast<clang::DeclRefExpr>(SymbolUse->getArg(0))) {
      std::string usedVar;
      if (clang::VarDecl* VD =
              clang::dyn_cast<clang::VarDecl>(DRE->getDecl())) {
        usedVar = VD->getQualifiedNameAsString();
      } else {
      }
      if (usedVar == var) {
        Found = true;
        return false;
      }
    }
    return true;
  }


  bool VisitExprWithCleanups(clang::ExprWithCleanups* SymbolUse) {
    if (auto* LHSexpr = SymbolUse->getSubExpr()) {
      std::string usedVar;
      if (clang::CXXMemberCallExpr* OME =
              clang::dyn_cast<clang::CXXMemberCallExpr>(LHSexpr)) {
        if (clang::DeclRefExpr* DRE = clang::dyn_cast<clang::DeclRefExpr>(
                OME->getImplicitObjectArgument())) {
          if (clang::VarDecl* VD =
                  clang::dyn_cast<clang::VarDecl>(DRE->getDecl())) {
            usedVar = VD->getQualifiedNameAsString();
          }
        }
      } else {
      }

      // llvm::outs() << "usedVar " << usedVar << " var "<< var <<"\n";

      if (usedVar == var) {
        Found = true;
        return false;
      }
    }
    return true;
  }


 private:
  bool Found = false;
  std::string var;
};


class MatchHandler : public clang::ast_matchers::MatchFinder::MatchCallback {
 public:
  using MatchResult = clang::ast_matchers::MatchFinder::MatchResult;

  explicit MatchHandler(unsigned Threshold) : Threshold(Threshold) {}

  void run(const MatchResult& Result) {
    const auto* Parameter =
        Result.Nodes.getNodeAs<clang::ParmVarDecl>("parameter");

    auto type = clang::QualType::getAsString(Parameter->getType().split());
    auto* Function = clang::cast<clang::FunctionDecl>(
        Parameter->getParentFunctionOrMethod());
    
    llvm::outs() << type <<"\n";
    // const clang::ASTRecordLayout &typeLayout(Parameter->getASTContext().getASTRecordLayout(Parameter));
    // llvm::outs() << "record '" << Parameter->getQualifiedNameAsString() << "' with " <<  typeLayout.getSize().getQuantity() << "bytes\n";

    bool param_value_changed = false;

    // llvm::outs() <<"Finding if param is changed : " <<

    if (LHSVariableFinder::find((clang::FunctionDecl*)Function,
                                Parameter->getQualifiedNameAsString())) {
      // llvm::outs() << "Param Values has changed";
      param_value_changed = true;
    }

    if (type.empty() || type.back() == '&' || type.back() == '*' || param_value_changed) return;
    auto& Diagnostics = Result.Context->getDiagnostics();
    const auto ID =
        Diagnostics.getCustomDiagID(clang::DiagnosticsEngine::Warning,
                                    "Parameter '%0' of function '%1' can be "
                                    "declared as reference");

    auto Builder = Diagnostics.Report(Parameter->getLocation(), ID);
    Builder.AddString(Parameter->getQualifiedNameAsString());
    Builder.AddString(Function->getQualifiedNameAsString());
  }

 private:
  unsigned Threshold;
};

class Consumer : public clang::ASTConsumer {
 public:
  template <typename... Args>
  explicit Consumer(Args&&... args) : Handler(std::forward<Args>(args)...) {
    using namespace clang::ast_matchers;
    const auto Matcher = parmVarDecl(isExpansionInMainFile()).bind("parameter");
    Finder.addMatcher(Matcher, &Handler);
  }

  void HandleTranslationUnit(clang::ASTContext& Context) {
    Finder.matchAST(Context);
  }

 private:
  MatchHandler Handler;
  clang::ast_matchers::MatchFinder Finder;
};

class Action : public clang::ASTFrontendAction {
 public:
  using ASTConsumerPointer = std::unique_ptr<clang::ASTConsumer>;

  explicit Action(unsigned Threshold) : Threshold(Threshold) {}

  ASTConsumerPointer
  CreateASTConsumer(clang::CompilerInstance&, llvm::StringRef) override {
    return std::make_unique<Consumer>(Threshold);
  }

  bool BeginSourceFileAction(clang::CompilerInstance& Compiler,
                             llvm::StringRef Filename) override {
    const auto& Language = Compiler.getLangOpts();

    // clang-format off
    llvm::outs() << "Processing '" << Filename
                 << "' (Signed overflow: " << Language.isSignedOverflowDefined()
                 << ")\n";
    // clang-format on

    return true;
  }

  void EndSourceFileAction() override {
    llvm::outs() << "\033[1mDone \033[91m<3\033[0m" << '\n';
  }

 private:
  unsigned Threshold;
};
}  // namespace McCabe

namespace {
llvm::cl::OptionCategory McCabeCategory("McCabe Options");

llvm::cl::extrahelp McCabeCategoryHelp(R"(
    Computes the McCabe (Cyclomatic) Complexity for each function in the given
    source files and emits a warning if the complexity is beyond a threshold.
)");

llvm::cl::opt<unsigned>
    ThresholdOption("threshold",
                    llvm::cl::init(2),
                    llvm::cl::desc("The threshold for emitting warnings"),
                    llvm::cl::cat(McCabeCategory));
llvm::cl::alias ShortThresholdOption("t",
                                     llvm::cl::desc("Alias for -threshold"),
                                     llvm::cl::aliasopt(ThresholdOption));

}  // namespace

struct ToolFactory : public clang::tooling::FrontendActionFactory {
  clang::FrontendAction* create() override {
    return new McCabe::Action(ThresholdOption);
  }
};

auto main(int argc, const char* argv[]) -> int {
  using namespace clang::tooling;

  CommonOptionsParser OptionsParser(argc, argv, McCabeCategory);
  ClangTool Tool(OptionsParser.getCompilations(),
                 OptionsParser.getSourcePathList());

  return Tool.run(new ToolFactory());
}
