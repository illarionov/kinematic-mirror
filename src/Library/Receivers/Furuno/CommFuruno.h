#ifndef COMMFURUNO_H_
#define COMMFURUNO_H_

#include "Comm.h"  
#include "Rs232.h"
#include "InputFile.h"


class CommFuruno: public Comm
{
public:
	CommFuruno(Stream& s);
	virtual ~CommFuruno();
	virtual bool GetBlock(Block& b);
	virtual bool PutBlock(Block& b);
	using Comm::PutBlock;

private:
	void CheckSum(Block& b, byte& ck_a, byte& ck_b);
	bool Open();
};


#endif /*COMMFURUNO_H_*/
