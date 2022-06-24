/* Authors: Jaein Lim */


#ifndef COLPA_DATASTRUCTURES_PATHCLASS_HPP_
#define COLPA_DATASTRUCTURES_PATHCLASS_HPP_


#include <array>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <iostream>

namespace colpa {
namespace datastructures {

/*
PathClass stores length of each colors in an array,
where the first index is the best class, and the last index is the worst.
*/

template<unsigned int DIM>
class PathClass : public std::array<double, DIM> {
public:
	PathClass() {}
	PathClass(double n){
		// std::fill(this->begin(),this->end(),std::numeric_limits<double>::infinity());
		std::fill(this->begin(),this->end(),n);
	}
	PathClass(const PathClass& s){std::copy(s.begin(),s.end(),this->begin());}
	PathClass operator + ( const PathClass& b) const{
		PathClass c;
		std::transform(this->begin(), this->end(), b.begin(), c.begin(),
				[](double a, double b){return a + b;});
		return c;
	}
	PathClass operator - ( const PathClass& b) const{
		PathClass c;
		std::transform(this->begin(), this->end(), b.begin(), c.begin(),
				[](double a, double b){return a - b;});
		return c;
	}
	PathClass operator * ( const double s) const{
		PathClass c;
		std::transform(this->begin(), this->end(), c.begin(),
				[s](double a)->double {return s*a;});
		return c;
	}

	bool operator == (const PathClass& b) const{
		for(unsigned i = DIM; i-- !=0; )
		{
			// You got here, only if i=DIM or all previous elements were equal
			if (std::abs(this->at(i)-b[i])>0.001) return false;
		}
		// You got here only if all elements are the same
		return true;
	}

	bool operator < (const PathClass& b) const{
		for(unsigned i = DIM; i-- !=0; )
		{
			// You got here, only if i=DIM or all previous elements were equal
			if (this->at(i) < b[i]) return true;
			if (this->at(i) > b[i]) return false;
		}
		// You got here only if all elements are the same
		return false;
	}

	bool operator <= (const PathClass& b) const{
		for(unsigned i = DIM; i-- !=0; )
		{
			// You got here, only if i=DIM or all previous elements were equal
			if (this->at(i) < b[i]) return true;
			if (this->at(i) > b[i]) return false;
		}
		// You got here only if all elements are the same
		return true;
	}

	double flatcost() const
	{
		double cost = 0;
		for(unsigned i = DIM; i-- !=0; )
		{
			cost += this->at(i);
		}
		return cost;
	}
	
/* Unused functions

	double normSq () const{
		return std::inner_product(this->begin(), this->end(), this->begin(), 0.0f);
	}
	double norm () const{
		return sqrt(normSq());
	}
	PathClass abs () const{
		PathClass s(*this);
		std::for_each(s.begin(),s.end(),[](double &a){a=fabs(a);});
		return s;
	}
	double normInf() const{
		PathClass s=this->abs();
		return *std::max_element(s.begin(),s.end());
	}

*/
	//print operator
	friend std::ostream& operator<< (std::ostream& stream, const PathClass& st) {
		stream << "(";
		for(int i=0;i<DIM-1;++i)
					stream << st[i] << ", ";
				stream << st[DIM-1] << ")";
		return stream;
	}
};


} // namespace datastructures
} // namespace colpa

#endif /* COLPA_DATASTRUCTURES_PATHCLASS_HPP_ */
