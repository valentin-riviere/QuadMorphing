#include "string_tools.h"

using namespace std;

bool numeric_string_compare(const std::string& s1, const std::string& s2)
{
    std::string::const_iterator it;

	int n1 = 0, n2 = 0;
	for (it = s1.begin() ; it != s1.end() ; it++)
	{
		if(std::isdigit(*it))
			n1++;
	}
	for (it = s2.begin() ; it != s2.end() ; it++)
	{
		if(std::isdigit(*it))
			n2++;
	}

	if (n1 != n2)
		return n1 < n2;
	else
	{
		int i = 0;
		for (it = s1.begin() ; it != s1.end() ; it++,i++)
		{
			if (s1[i] != s2[i])
			{
				return s1[i] < s2[i];
			}
		}

	}

	return false;
}
