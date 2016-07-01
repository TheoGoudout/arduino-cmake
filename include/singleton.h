#ifndef SINGLETON_H_INCLUDED
#define SINGLETON_H_INCLUDED

template <class classT>
class Singleton
{
public:
	static classT& instance()
	{
		if (mPtr == NULL)
			mPtr = new classT(); 

		return *mPtr;
	}

	static void destroy()
	{
		delete mPtr();

		mPtr = NULL;
	}

private:
	static classT* mPtr;
};

template <class classT>
classT* Singleton<classT>::mPtr = NULL;

#endif /* SINGLETON_H_INCLUDED */