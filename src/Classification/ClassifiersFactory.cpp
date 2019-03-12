#include "ClassifiersFactory.h"
#include "DiffusedNormalVotingClassifier.h"
#include "CovarianceMatrixClassifier.h"
#include "Tensor3dvoting.h"

std::map<ClassifierTypes, Classifier*> ClassifiersFactory::_classifierInstanceMap;

ClassifiersFactory::ClassifiersFactory()
{
}

ClassifiersFactory::~ClassifiersFactory()
{
}

Classifier* ClassifiersFactory::GetInstance(ClassifierTypes classifierType)
{
	std::map<ClassifierTypes, Classifier*>::iterator it;
	it = ClassifiersFactory::_classifierInstanceMap.find(classifierType);

	if (it != ClassifiersFactory::_classifierInstanceMap.end())
		return it->second;

	Classifier* instance = ClassifiersFactory::GetClassifier(classifierType);
	ClassifiersFactory::_classifierInstanceMap.insert(
		std::pair<ClassifierTypes, Classifier*>(classifierType, instance)
	);

	return instance;
}

Classifier* ClassifiersFactory::GetClassifier(ClassifierTypes classifierType)
{
	Classifier* object;
	switch (classifierType)
	{
	case C_3DVTGET:
		object = new DiffusedNormalVotingClassifier();
		break;
	case C_3DVT:
		object = new DiffusedNormalVotingClassifier();
		break;
	case C_3DCM:
		object = new CovarianceMatrixClassifier();
		break;
	case C_3DMCM:
		object = new CovarianceMatrixClassifier();
		break;
	case C_2DGET:
		object = new CovarianceMatrixClassifier();
		break;
	case C_Hessian:
		object = new CovarianceMatrixClassifier();
		break;
	case C_2DCM:
		object = new CovarianceMatrixClassifier();
		break;
	default:
		object = new CovarianceMatrixClassifier();
	}

	return object;
}