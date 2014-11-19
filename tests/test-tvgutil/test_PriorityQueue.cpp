/**
 * test-tvgutil: test_PriorityQueue.cpp
 */

#include <boost/test/unit_test.hpp>

#include <tvgutil/PriorityQueue.h>
using namespace tvgutil;

typedef	PriorityQueue<std::string, double, int> PQ;//, std::greater<double> > PQ;

BOOST_AUTO_TEST_SUITE(test_PriorityQueue)

BOOST_AUTO_TEST_CASE(clear_test)
{
	PQ pq;
	pq.insert("S", 1.0, 23);
		BOOST_CHECK_EQUAL(pq.empty(), false);
	pq.clear();
		BOOST_CHECK_EQUAL(pq.empty(), true);
}

BOOST_AUTO_TEST_CASE(contains_test)
{
	PQ pq;
		BOOST_CHECK_EQUAL(pq.contains("S"), false);
		BOOST_CHECK_EQUAL(pq.contains("K"), false);
	pq.insert("S", 1.0, 23);
		BOOST_CHECK_EQUAL(pq.contains("S"), true);
		BOOST_CHECK_EQUAL(pq.contains("K"), false);
	pq.insert("K", 0.9, 13);
		BOOST_CHECK_EQUAL(pq.contains("S"), true);
		BOOST_CHECK_EQUAL(pq.contains("K"), true);
}

BOOST_AUTO_TEST_CASE(element_test)
{
	PQ pq;
	pq.insert("S", 1.0, 23);
	PQ::Element& e = pq.element("S");
		BOOST_CHECK_EQUAL(e.id(), "S");
		BOOST_CHECK_CLOSE(e.key(), 1.0, 0.001);	// they differ by no more than 0.001% of their value
		BOOST_CHECK_EQUAL(e.data(), 23);
}

// Note: empty() has been tested in other test cases

BOOST_AUTO_TEST_CASE(erase_test)
{
	PQ pq;
		BOOST_CHECK_EQUAL(pq.size(), 0);
	pq.insert("S", 1.0, 23);
		BOOST_CHECK_EQUAL(pq.size(), 1);
		BOOST_CHECK_EQUAL(pq.contains("S"), true);
	pq.erase("S");
		BOOST_CHECK_EQUAL(pq.size(), 0);
		BOOST_CHECK_EQUAL(pq.contains("S"), false);
}

// Note: insert() has been tested in other test cases

BOOST_AUTO_TEST_CASE(pop_test)
{
	PQ pq;
	pq.insert("S", 1.0, 23);
	pq.insert("K", 0.9, 13);
		BOOST_CHECK_EQUAL(pq.contains("S"), true);
		BOOST_CHECK_EQUAL(pq.contains("K"), true);
	pq.pop();
		BOOST_CHECK_EQUAL(pq.contains("S"), false);
		BOOST_CHECK_EQUAL(pq.contains("K"), true);
}

BOOST_AUTO_TEST_CASE(size_test)
{
	PQ pq;
		BOOST_CHECK_EQUAL(pq.size(), 0);
	pq.insert("S", 1.0, 23);
		BOOST_CHECK_EQUAL(pq.size(), 1);
	pq.insert("K", 0.9, 13);
		BOOST_CHECK_EQUAL(pq.size(), 2);
	pq.insert("M", 1.1, 7);
		BOOST_CHECK_EQUAL(pq.size(), 3);
	pq.insert("D", 1.1, 7);
		BOOST_CHECK_EQUAL(pq.size(), 4);
	pq.insert("G", 1.1, 24);
		BOOST_CHECK_EQUAL(pq.size(), 5);
	pq.erase("S");
		BOOST_CHECK_EQUAL(pq.size(), 4);
	pq.insert("S", 1.0, 23);
		BOOST_CHECK_EQUAL(pq.size(), 5);

	for(int i=0; i<5; ++i)
	{
		pq.pop();
		BOOST_CHECK_EQUAL(pq.size(), 4 - i);
	}
}

BOOST_AUTO_TEST_CASE(top_test)
{
	PQ pq;
	pq.insert("S", 1.0, 23);
	pq.insert("K", 1.1, 13);
		BOOST_CHECK_EQUAL(pq.top().id(), "K");
	pq.pop();
		BOOST_CHECK_EQUAL(pq.top().id(), "S");
}

BOOST_AUTO_TEST_CASE(update_key_test)
{
	PQ pq;
	pq.insert("S", 1.0, 23);
	pq.insert("M", 0.9, 7);
	pq.update_key("M", 1.1);
		BOOST_CHECK_EQUAL(pq.top().id(), "M");
		BOOST_CHECK_CLOSE(pq.top().key(), 1.1, 0.001);
		BOOST_CHECK_EQUAL(pq.top().data(), 7);
	pq.pop();
		BOOST_CHECK_EQUAL(pq.top().id(), "S");
		BOOST_CHECK_CLOSE(pq.top().key(), 1.0, 0.001);
		BOOST_CHECK_EQUAL(pq.top().data(), 23);
	pq.pop();
		BOOST_CHECK_EQUAL(pq.empty(), true);
}

BOOST_AUTO_TEST_SUITE_END()
