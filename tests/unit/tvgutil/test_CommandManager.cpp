#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;

#include <tvgutil/commands/CommandManager.h>
#include <tvgutil/commands/SeqCommand.h>
using namespace tvgutil;

struct TestCommand : Command
{
  std::string m_executeText;
  std::string& m_output;
  std::string m_undoText;

  TestCommand(std::string& output, const std::string& executeText, const std::string& undoText, const std::string& description)
  : Command(description), m_executeText(executeText), m_output(output), m_undoText(undoText)
  {}

  virtual void execute() const
  {
    m_output += m_executeText;
  }

  virtual void undo() const
  {
    m_output += m_undoText;
  }
};

BOOST_AUTO_TEST_SUITE(test_CommandManager)

BOOST_AUTO_TEST_CASE(basic_test)
{
  std::string output;

  Command_CPtr c1(new TestCommand(output, "E1", "U1", ""));
  Command_CPtr c2(new TestCommand(output, "E2", "U2", ""));
  Command_CPtr c3(new TestCommand(output, "E3", "U3", ""));

  CommandManager cm;

  // Test a simple sequence of operations to make sure that the command manager's working as expected.
  cm.execute_command(c1);
    BOOST_CHECK_EQUAL(output, "E1");
  cm.execute_command(c2);
    BOOST_CHECK_EQUAL(output, "E1E2");
    BOOST_CHECK_EQUAL(cm.can_undo(), true);
    BOOST_CHECK_EQUAL(cm.can_redo(), false);
  cm.undo();
    BOOST_CHECK_EQUAL(output, "E1E2U2");
    BOOST_CHECK_EQUAL(cm.can_redo(), true);
  cm.redo();
    BOOST_CHECK_EQUAL(output, "E1E2U2E2");
    BOOST_CHECK_EQUAL(cm.can_redo(), false);
  cm.execute_command(c3);
    BOOST_CHECK_EQUAL(output, "E1E2U2E2E3");
    BOOST_CHECK_EQUAL(cm.can_undo(), true);
  cm.undo();
    BOOST_CHECK_EQUAL(output, "E1E2U2E2E3U3");
    BOOST_CHECK_EQUAL(cm.can_redo(), true);
  cm.execute_command(c1);
    BOOST_CHECK_EQUAL(output, "E1E2U2E2E3U3E1");
    BOOST_CHECK_EQUAL(cm.can_undo(), true);
    BOOST_CHECK_EQUAL(cm.can_redo(), false);
  cm.undo();
    BOOST_CHECK_EQUAL(output, "E1E2U2E2E3U3E1U1");
    BOOST_CHECK_EQUAL(cm.can_undo(), true);
  cm.undo();
    BOOST_CHECK_EQUAL(output, "E1E2U2E2E3U3E1U1U2");
    BOOST_CHECK_EQUAL(cm.can_undo(), true);
  cm.undo();
    BOOST_CHECK_EQUAL(output, "E1E2U2E2E3U3E1U1U2U1");
    BOOST_CHECK_EQUAL(cm.can_undo(), false);
    BOOST_CHECK_EQUAL(cm.can_redo(), true);
  cm.reset();
    BOOST_CHECK_EQUAL(cm.can_redo(), false);
}

BOOST_AUTO_TEST_CASE(compressible_test)
{
  std::string output;

  Command_CPtr begin(new TestCommand(output, "Eb", "Ub", "Begin"));
  Command_CPtr middle(new TestCommand(output, "Em", "Um", "Middle"));
  Command_CPtr end(new TestCommand(output, "Ee", "Ue", "End"));

  std::map<std::string,std::string> precursors = map_list_of("Begin","Middle")("Middle","Middle");

  CommandManager cm;

  cm.execute_command(begin);
    BOOST_CHECK_EQUAL(output, "Eb");
    BOOST_CHECK_EQUAL(cm.executed_count(), 1);
  cm.execute_compressible_command(middle, precursors);
    BOOST_CHECK_EQUAL(output, "EbEm");
    BOOST_CHECK_EQUAL(cm.executed_count(), 1);
  cm.execute_compressible_command(middle, precursors);
    BOOST_CHECK_EQUAL(output, "EbEmEm");
    BOOST_CHECK_EQUAL(cm.executed_count(), 1);
  cm.execute_compressible_command(end, precursors);
    BOOST_CHECK_EQUAL(output, "EbEmEmEe");
    BOOST_CHECK_EQUAL(cm.executed_count(), 1);
    BOOST_CHECK_EQUAL(cm.undone_count(), 0);
  cm.undo();
    BOOST_CHECK_EQUAL(output, "EbEmEmEeUeUmUmUb");
    BOOST_CHECK_EQUAL(cm.executed_count(), 0);
    BOOST_CHECK_EQUAL(cm.undone_count(), 1);
  cm.redo();
    BOOST_CHECK_EQUAL(output, "EbEmEmEeUeUmUmUbEbEmEmEe");
    BOOST_CHECK_EQUAL(cm.executed_count(), 1);
    BOOST_CHECK_EQUAL(cm.undone_count(), 0);
  cm.execute_command(begin);
    BOOST_CHECK_EQUAL(output, "EbEmEmEeUeUmUmUbEbEmEmEeEb");
    BOOST_CHECK_EQUAL(cm.executed_count(), 2);
    BOOST_CHECK_EQUAL(cm.undone_count(), 0);
  cm.execute_compressible_command(middle, precursors);
    BOOST_CHECK_EQUAL(output, "EbEmEmEeUeUmUmUbEbEmEmEeEbEm");
    BOOST_CHECK_EQUAL(cm.executed_count(), 2);
    BOOST_CHECK_EQUAL(cm.undone_count(), 0);
  cm.execute_compressible_command(end, precursors);
    BOOST_CHECK_EQUAL(output, "EbEmEmEeUeUmUmUbEbEmEmEeEbEmEe");
    BOOST_CHECK_EQUAL(cm.executed_count(), 2);
    BOOST_CHECK_EQUAL(cm.undone_count(), 0);
  cm.undo();
    BOOST_CHECK_EQUAL(output, "EbEmEmEeUeUmUmUbEbEmEmEeEbEmEeUeUmUb");
    BOOST_CHECK_EQUAL(cm.executed_count(), 1);
    BOOST_CHECK_EQUAL(cm.undone_count(), 1);
}

BOOST_AUTO_TEST_CASE(error_test)
{
  std::string output;

  Command_CPtr c1(new TestCommand(output, "E1", "U1", ""));
  Command_CPtr c2(new TestCommand(output, "E2", "U2", ""));
  Command_CPtr c3(new TestCommand(output, "E3", "U3", ""));
  Command_CPtr nc;

  // Test that we can only construct a sequence command from two "smaller" commands if they're both non-NULL.
  BOOST_CHECK_THROW(SeqCommand(nc, nc, ""), std::runtime_error);
  BOOST_CHECK_THROW(SeqCommand(c1, nc, ""), std::runtime_error);
  BOOST_CHECK_THROW(SeqCommand(nc, c2, ""), std::runtime_error);
  BOOST_CHECK_NO_THROW(SeqCommand(c1, c2, ""));

  // Test that we can only construct a sequence command from a sequence of "smaller" commands if they're all non-NULL.
  BOOST_CHECK_THROW(SeqCommand(list_of(c1)(nc)(c3), ""), std::runtime_error);
  BOOST_CHECK_NO_THROW(SeqCommand(list_of(c1)(c2)(c3), ""));
}

BOOST_AUTO_TEST_CASE(seq_test)
{
  std::string output;

  Command_CPtr c1(new TestCommand(output, "E1", "U1", ""));
  Command_CPtr c2(new TestCommand(output, "E2", "U2", ""));
  Command_CPtr c3(new TestCommand(output, "E3", "U3", ""));

  Command_CPtr c12(new SeqCommand(c1, c2, ""));
  Command_CPtr c123(new SeqCommand(list_of(c1)(c2)(c3), ""));

  CommandManager cm;

  // Test that the "smaller" commands inside a sequence command are executed and then undone in the right order.
  cm.execute_command(c12);
    BOOST_CHECK_EQUAL(output, "E1E2");
  cm.undo();
    BOOST_CHECK_EQUAL(output, "E1E2U2U1");

  output = "";
  cm.reset();
  cm.execute_command(c123);
    BOOST_CHECK_EQUAL(output, "E1E2E3");
  cm.undo();
    BOOST_CHECK_EQUAL(output, "E1E2E3U3U2U1");
}

BOOST_AUTO_TEST_SUITE_END()
