#include "../data_type/planning_data_type.h"
#include "../data_type/poly.h"

class candidate_builder{
    public:
	candidate_builder();
	~candidate_builder();

    private:
	void GenerateVariantsS();
	void GenerateVariantsN();

	void CheckFeasibleS();
	void CheckFeasibleN();
};
