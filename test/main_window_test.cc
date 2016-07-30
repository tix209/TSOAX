#include "gui/main_window.h"

#include "gtest/gtest.h"
// #include <QString>

namespace {

class MainWindowTest : public ::testing::Test {
 protected:
  MainWindowTest() {}
  // virtual ~MainWindowTest() {}
};

TEST_F(MainWindowTest, ExtractDirWorks) {
  soax::MainWindow w;

}

} // namespace


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
