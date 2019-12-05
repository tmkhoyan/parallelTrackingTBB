/*
 * Description:    Parallel tracker and feature extraction method initial release
 *
 * Author:          Tigran Mkhoyan
 * Email :          t.mkhoyan@tudelft.nl
 */



/* ----------------------------------- mat >> point ----------------------------------- */

//convert mat to float point 
inline std::vector<cv::Point2f> mat2point2f(cv::Mat m){
  	cv::MatIterator_<float> it;
	std::vector <cv::Point2f> v; v.reserve(m.rows);
	for(it = m.begin<float>();it != m.end<float>();++it){
		auto x = (*it); it++; // assign x and advance
		auto y = (*it); //asign y
		v.push_back(cv::Point2f(x,y));
	}
	return v;
}

//convert mat to float point 
inline std::vector<cv::Point2d> mat2point2d(cv::Mat m){
    cv::MatIterator_<double> it;
  std::vector <cv::Point2d> v; v.reserve(m.rows);
  for(it = m.begin<double>();it != m.end<double>();++it){
    auto x = (*it); it++; // assign x and advance
    auto y = (*it); //asign y
    v.push_back(cv::Point2d(x,y));
  }
  return v;
}

/* ---------------- typefree ------ */
//convert any mat to point2 (x,y) type. Iterator type is needed to infer the return vector type!
template <typename T>
inline std::vector<cv::Point_<T>> mat2Dpoint(cv::MatIterator_<T> it,cv::Mat m){
  // cv::MatIterator_<T> it;
  std::vector <cv::Point_<T>> v; v.reserve(m.rows);
  for(it = m.begin<T>();it != m.end<T>();++it){
    auto x = (*it); it++; // assign x and advance
    auto y = (*it); //asign y
    v.push_back(cv::Point_<T>(x,y));
  }
  return v;
}

//convert any mat to point3 (x,y,z) type. Iterator type is needed to infer the return vector type!
template <typename T>
inline std::vector<cv::Point3_<T>> mat3Dpoint(cv::MatIterator_<T> it,cv::Mat m){
  // cv::MatIterator_<T> it;
  std::vector <cv::Point3_<T>> v; v.reserve(m.rows);
  for(it = m.begin<T>();it != m.end<T>();++it){
    auto x = (*it); it++; // assign x and advance
    auto y = (*it); it++; //asign y
    auto z = (*it); //asign y
    v.push_back(cv::Point3_<T>(x,y,z));
  }
  return v;
}

//convert any mat to any point2 type. Iterator type is not needed if template is called properly
template <typename T>
inline std::vector<T> mat2point(cv::Mat m){
  std::vector <T> v; v.reserve(m.rows);
  cv::MatConstIterator_<T> it = m.begin<T>(), it_end = m.end<T>();
  for(; it != it_end; it+=m.cols){
    v.push_back((*it));
  }
  return v;
}

/* ----------------------------------- mat << point----------------------------------- */

//convert float point type to 2d mat
inline cv::Mat point2f2mat(std::vector<cv::Point2f> p){
	return cv::Mat(p.size(),2,CV_32F,p.data());
}

//convert float point type to 3d mat
inline cv::Mat point3f2mat(std::vector<cv::Point3f> p){
	return cv::Mat(p.size(),3,CV_32F,p.data());
}

//convert any point type to 2d mat
template <typename T>
inline cv::Mat point2Dmat(std::vector< cv::Point_<T> > p){
  return cv::Mat(p.size(),2,cv::DataType<T>::type,p.data());
}

//convert any point type to 3d mat
template <typename T>
inline cv::Mat point3Dmat(std::vector< cv::Point3_<T> > p){
  return cv::Mat(p.size(),3,cv::DataType<T>::type,p.data());
}

/* ----------------------------------- keypoint >> point----------------------------------- */

template <typename T>
inline std::vector<cv::Point_<T>> key2point(const std::vector<cv::KeyPoint> &keypoint){
  std::vector<cv::Point_<T>> p; p.reserve(keypoint.size());
  std::for_each(keypoint.begin(),keypoint.end(),
    [&p](const cv::KeyPoint &kp){p.push_back(kp.pt);});
  return p;
}

/* ----------------------------------- matrix manupulation ----------------------------------- */

inline cv::Mat rotateImg90(cv::Mat image){
  cv::transpose(image, image);
  cv::flip(image, image, +1);
  return image;
}
  // transpose(matImage, matImage);  
 //    flip(matImage, matImage,1); //transpose+flip(1)=CW
 //  } else if (rotflag == 2) {
 //    transpose(matImage, matImage);  
 //    flip(matImage, matImage,0); //transpose+flip(0)=CCW     
 //  } else if (rotflag ==3){
 //    flip(matImage, matImage,-1);    //flip(-1)=180          
 //  } else if (rotflag != 0){ //if not 0,1,2,3:

/* ----------------------------------- print methods ----------------------------------- */

template <typename T>
inline void printMatCol(cv::MatIterator_<T> it, cv::Mat m, size_t col=0){
  int k(0);
  for(it = (m.begin<T>())+col;it != m.end<T>(); it+=m.cols){
    std::cout << "(" << k++ <<"," << col << ") = " << (*it) << std::endl; 
  }
}

template <typename T>
inline void printVector(const std::vector <T> &v){
  int k =0;
  std::for_each(v.begin(),v.end(),
  [&k](const T &s){std::cout << k++ << ": " << s << std::endl;});
}

template <typename T>
inline void printRowVector(const std::vector <T> &v){
  std::cout<< "[" ; for(auto p: v){std::cout << p << ", " ;} std::cout << "]" << std::endl;
}

template <typename T, typename S>
    inline void printMap(const std::map<T,S> m){
      int k =0;
      // for(auto p: m)
      //  std::cout << k++ << ": " <<p.first << ", "<< p.second << std::endl;
      std::for_each(m.begin(),m.end(),
          [&k](std::pair<const T,S> index){std::cout << k++ <<": " <<index.first << ", \t " <<index.second << std::endl;});
    }


//  ------------------- new test if works

template<typename T>
T getModeDat(std::vector<T> dat, size_t dec=0){  // get the most frequent value in data
std::unordered_map<T,int> datHist;
    if(dec==0){
        for(auto p: dat)
            ++datHist[p];
    } else{
        int dec_ = pow(10,dec);
        for(auto p: dat)
            ++datHist[round(p*dec_)/dec_];
    }

    // sort only most frequent element
    std::vector<std::pair<T, int>> pairs;  pairs.reserve(datHist.size());
    std::copy(datHist.begin(), datHist.end(), std::back_inserter(pairs));
    std::partial_sort(pairs.begin(), pairs.begin()+1, pairs.end(),
          [](const std::pair<T, int> &a, const std::pair<T, int> &b) {return a.second > b.second;});

    auto it = pairs.begin();
    return (it->first);
}
    
template<typename T>
std::vector<T> getMatchingElements(const std::vector<T> &ItemsL, const std::vector<T> &ItemsR,  std::vector<int > &matchedIndexL, std::vector<int> &matchedIndexR){
  std::vector<T> matchedItems;
  std::vector<int> matchedIndexL_, matchedIndexR_;

// std::vector<T>::iterator 
  
   for(auto iteratorL =  ItemsL.begin(); iteratorL != ItemsL.end(); iteratorL++){
    // look for matching marker id
    auto iteratorR = std::find(ItemsR.begin(),ItemsR.end(),*iteratorL);
    // if found
    if(iteratorR != ItemsR.end()){
      int indexL = std::distance(ItemsL.begin(),iteratorL); //current id of left in the id list
      int indexR = std::distance(ItemsR.begin(),iteratorR); //current id of right

      matchedItems.push_back(*iteratorL); // matched id marker to laser
      matchedIndexL_.push_back(indexL);      // matched index of of marker in left
      matchedIndexR_.push_back(indexR);      // matched index of of marker in right
    }
  }

  matchedIndexL = matchedIndexL_; //std::move(matchedIndexL_);
  matchedIndexR = matchedIndexR_; //std::move(matchedIndexR_);

  return matchedItems;
}


template<typename T>
std::map<T,std::pair<int,int>> getMatchingElementsSortedMap(const std::vector<T> &ItemsL, const std::vector<T> &ItemsR){
  std::map<T,std::pair<int,int>> matchedItemMap;
// std::vector<T>::iterator 
   for(auto iteratorL =  ItemsL.begin(); iteratorL != ItemsL.end(); iteratorL++){
    // look for matching marker id
    auto iteratorR = std::find(ItemsR.begin(),ItemsR.end(),*iteratorL);
    // if found
    if(iteratorR != ItemsR.end()){
      int indexL = std::distance(ItemsL.begin(),iteratorL); //current id of left in the id list
      int indexR = std::distance(ItemsR.begin(),iteratorR); //current id of right

      matchedItemMap[*iteratorL] = std::make_pair(indexL,indexR);
    }
  }
  return matchedItemMap;
}



      template <typename T>
    inline std::map<T,unsigned int> pushVectoMap(const std::vector <T> &v){
      std::map<T, unsigned int> nameMap;
      int k = 0;
      std::for_each(std::begin(v), std::end(v),
          [&nameMap,&k](const T& s) { nameMap[s]=k++;});
      return nameMap;
    }
  //push map into vector
  template <typename T,typename S>
    inline  std::vector<T> pushMapFirsttoVector(const std::map<T,S> map_in){
      std::vector <T> vec;
      std::for_each(map_in.begin(),map_in.end(),
          [&vec](std::pair<T,S> index){vec.push_back(index.first);});
      return vec;
    }
  template <typename T,typename S>
  inline  std::vector<S> pushMapSecondtoVector(const std::map<T,S> map_in){
    std::vector <S> vec;
    std::for_each(map_in.begin(),map_in.end(),
        [&vec](std::pair<T,S> index){vec.push_back(index.second);});
    return vec;
  }

  template <typename T>
  inline  std::vector<T> reorderVector(const vector<T> &v,const vector<unsigned int> &indexVector){
    std::vector<T> sorted;
    std::for_each(indexVector.begin(),indexVector.end(),
      [&v,&sorted](const unsigned int & index){sorted.push_back(v[index]);});
    return sorted;
  }

template <typename T>
vector<size_t> sortIdx(const vector<T> &v) {
  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);
  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}


