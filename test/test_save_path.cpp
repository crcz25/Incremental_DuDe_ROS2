// H-01: the file name of a saved image comes from untrusted input (the save
// trigger). savePath() must reject any name that could leave the output
// directory, and accept plain names.

#include <gtest/gtest.h>

#include <filesystem>
#include <optional>
#include <string>

#include "inc_dude/save_path.hpp"

namespace {

namespace fs = std::filesystem;

class SavePath : public ::testing::Test {
protected:
  void SetUp() override {
    root_ = fs::path(::testing::TempDir()) /
            ("inc_dude_save_path_" + std::string(::testing::UnitTest::GetInstance()
                                                     ->current_test_info()
                                                     ->name()));
    fs::remove_all(root_);
    out_ = root_ / "out";
    fs::create_directories(out_);
  }
  void TearDown() override { fs::remove_all(root_); }

  std::optional<std::string> path(const std::string &name) {
    return inc_dude::savePath(out_.string(), name, "_Batch.png");
  }

  fs::path root_, out_;
};

TEST_F(SavePath, AcceptsAPlainName) {
  const auto p = path("run_01-a.b");
  ASSERT_TRUE(p);
  EXPECT_EQ(fs::path(*p), fs::weakly_canonical(out_ / "run_01-a.b_Batch.png"));
  EXPECT_EQ(fs::path(*p).parent_path(), fs::weakly_canonical(out_));
}

TEST_F(SavePath, AcceptsADirectoryWithATrailingSeparator) {
  // The directory need not exist yet (the write then reports the error).
  const fs::path missing = root_ / "missing";
  const auto p =
      inc_dude::savePath(missing.string() + "/", "run", "_Batch.png");
  ASSERT_TRUE(p);
  EXPECT_EQ(fs::path(*p), fs::weakly_canonical(missing / "run_Batch.png"));
}

TEST_F(SavePath, RejectsNamesThatLeaveTheOutputDirectory) {
  for (const std::string name : {"../x", "/etc/x", "a/../../x", "..", ".", "a..b",
                                 "..x", "x/y", "x\\y", "C:\\x"}) {
    EXPECT_FALSE(path(name)) << "'" << name << "'";
  }
}

TEST_F(SavePath, RejectsEmptyNulLongAndUnexpectedCharacters) {
  EXPECT_FALSE(path(""));
  EXPECT_FALSE(path(std::string("a\0b", 3)));
  EXPECT_FALSE(path(std::string(200, 'a')));
  EXPECT_FALSE(path(std::string(129, 'a')));
  EXPECT_TRUE(path(std::string(128, 'a')));
  for (const std::string name : {"a b", "a;b", "a$b", "a*b", "a:b", "\xc3\xa9t\xc3\xa9"}) {
    EXPECT_FALSE(path(name)) << "'" << name << "'";
  }
}

// A link inside the output directory that points outside it is not
// followed out of it.
TEST_F(SavePath, RejectsALinkThatPointsOutside) {
  fs::create_directories(root_ / "elsewhere");
  fs::create_symlink(root_ / "elsewhere" / "target.png", out_ / "link_Batch.png");
  EXPECT_FALSE(path("link"));
}

}  // namespace
