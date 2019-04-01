//#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
//#include "opencv2/core/core.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <iostream>
#include <queue>


using namespace cv;
using namespace std;

Mat image;

struct state{
	float vel;
	float ang_vel;
	float orientation;
	float x;
	float y;
};

float alpha = 10,beta = 0,gamm = 0;

int bound(int x,int limit){
	if(x<0) return 0;
	else if(x>=limit) return limit-1;
	else return x;
}

bool isValid(float x,float y,float theta,int length,int bredth){
	float t = -theta;
	if(image.at<Vec3b>(bound(x+length*cos(t*3.14/180)-bredth*sin(t*3.14/180),image.rows),bound(y+length*sin(t*3.14/180)+bredth*cos(t*3.14/180),image.cols))[0]==255 || image.at<Vec3b>(bound(x+length*cos((t+90)*3.14/180)-bredth*sin((t+90)*3.14/180),image.rows),bound(y+length*sin((t+90)*3.14/180)+bredth*cos((t+90)*3.14/180),image.cols))[0]==255 || image.at<Vec3b>(bound(x+length*cos((t+180)*3.14/180)-bredth*sin((t+180)*3.14/180),image.rows),bound(y+length*sin((t+180)*3.14/180)+bredth*cos((t+180)*3.14/180),image.cols))[0]==255 || image.at<Vec3b>(bound(x+length*cos((t+270)*3.14/270)-bredth*sin((t+270)*3.14/270),image.rows),bound(y+length*sin((t+270)*3.14/270)+bredth*cos((t+270)*3.14/180),image.cols))[0]==255)
	{
		return false;
	}
	else return true;
}

int sgn(int x){
	if(x>0) return 1;
	else if(x<0) return -1;
	else return 0;

}

float calc(int x,int y,int orientation,int j,float r,Point end){
	int y1 = y-r * sin((orientation - 0.25*j)*(3.14/180)) + r*sin((orientation)*(3.14/180));
	int x1 = x - r * cos((orientation - 0.25*j)*(3.14/180)) + r * cos((orientation)*(3.14/180));
	orientation -= 0.25 * j;
	float theta = atan(-(float)(end.x-x)/(end.y-y))*(180/3.14);
	if(end.y<y) theta = (int)(theta + 180)%360; 
	float a;
	//if(sgn(r*j)<0) orientation = ((int)orientation + 180)%360; 
	if(theta-orientation < -180) a = theta-orientation + 360;
	else if(theta-orientation > 180) a = theta-orientation - 360;
	else a = theta-orientation;
	//cout << 180 - abs(a) << " " << j<<endl;
	return ( alpha *(180 - abs(a)) + beta * r*j);
}
					
class PathPlanner
{
	public:
		PathPlanner(Point star, Point en, Mat ma,int len,int bred){
			start = star;
			length = len;
			bredth = bred;
			end = en;
			map = ma; 
			acceleration=8;
			ang_acceleration = 40;
		}
		void getPath(){
			curr_state.vel = 0;
			curr_state.ang_vel = 0;
			curr_state.x = start.x;
			curr_state.y = start.y;
			cout << curr_state.x << " " << curr_state.y <<endl;
			curr_state.orientation = 0;
			
			float r;
			while(!(curr_state.x==end.x&&curr_state.y==end.y)){
				float dist,penalty,i,j,velocity=curr_state.vel,ang_velocity = curr_state.ang_vel,x=curr_state.x,y = curr_state.y,orientation = curr_state.orientation;
				float maxval = -100; int maxi = 0; int maxj = 0;
				bool allnotpossible = true;
				for(i=velocity+acceleration/4;i>=velocity-acceleration/4;i--){
					//cout << velocity << endl;
					if(i==0||abs(i)>25||i<0) continue;

					for(j=ang_velocity-ang_acceleration/4;j<=ang_velocity+ang_acceleration/4;j++){
						//cout << j << endl;
						if(j!=0||abs(j)>100) r = (float)i/j * 180/3.14;
						else continue;
						for(float t=0;true;t+=0.25*sgn(j)){
							//cout << "count" << t <<endl;
							float y1 = y-r * sin((orientation - t)*(3.14/180)) + r*sin((orientation)*(3.14/180));
							float x1 = x - r * cos((orientation - t)*(3.14/180)) + r * cos((orientation)*(3.14/180));
							float theta = (orientation - t); 
							if(!isValid(x1,y1,theta,length,bredth)) {
								if(abs(t)<abs(j*0.25)) {
									//cout <<"abe"<<endl;
									break;
								}
								dist = r * (t) * (3.14/180); 
								//cout << dist << "oh" << t <<endl;
								if(abs(i)<=sqrt(acceleration*dist*sgn(dist))-10){
									if((calc(x,y,orientation,j,r,end) + gamm * dist ) > maxval){
										//cout << t <<" s"<<endl;
										allnotpossible = false;
										maxval = (calc(x,y,orientation,j,r,end) + gamm * dist );
										maxi = i;
										maxj = j;
										//cout << "he" << maxj<<endl;
									}
								}
								else{
									//cout << sqrt(2*acceleration*dist*sgn(dist)) << " is the not possible velocity & " << i << "is" << j << " the " << t << " not possible angle "<<endl;
									//return;
								}
								break;
							}
							if(t>=360||t<=-360){
								//cout << r << "o"<< endl;
								//cout <<"what"<<endl;
								if(calc(x,y,orientation,j,r,end) + abs(gamm * 2 * 3.14 * r ) > maxval){
									//cout << "yes" << endl;
									allnotpossible = false;
									maxval = calc(x,y,orientation,j,r,end) + abs(gamm * 2 * 3.14 * r );
									maxi = i;
									maxj = j;
									//cout << "he" << maxj<<endl;
								}
								break;
							}
						}
					}
				}
				if(allnotpossible) {
					cout << "All of the possible options are not viable" << endl;
					//return;
				}
				//cout << "hehe" << maxj << " " << sgn(maxj) * (orientation-maxj*0.25)<<endl;
				int yd = y;int xd = x;
				for(float t=orientation;sgn(maxj)*t>=sgn(maxj) * (orientation-maxj*0.25);t= t - sgn(maxj)){
					r = (float)maxi/maxj*180/3.14;
					y = yd + r*sin(orientation*(3.14/180)) - r*sin((3.14/180)*(t));
					x = xd + r*cos(orientation*(3.14/180)) - r*cos((3.14/180)*(t));
					//cout << maxi <<endl;
					//cout << maxj << endl;
					//cout << x+length*sin(t*3.14/180)+bredth*cos(t*3.14/180) << " " << y+length*cos(t*3.14/180)-bredth*sin(t*3.14/180) <<endl;
					//return;
					
					Point vertices[4];
					vertices[0] = Point(y+length*cos(t*3.14/180)+bredth*sin(t*3.14/180),x-length*sin(t*3.14/180)+bredth*cos(t*3.14/180));
					vertices[1] = Point(y+length*cos((-t+90)*3.14/180)-bredth*sin((-t+90)*3.14/180),x+length*sin((-t+90)*3.14/180)+bredth*cos((-t+90)*3.14/180));
					vertices[2] = Point(y+length*cos((-t+180)*3.14/180)-bredth*sin((-t+180)*3.14/180),x+length*sin((-t+180)*3.14/180)+bredth*cos((-t+180)*3.14/180));
					vertices[3] = Point(y+length*cos((-t+270)*3.14/180)-bredth*sin((-t+270)*3.14/180),x+length*sin((-t+270)*3.14/180)+bredth*cos((-t+270)*3.14/180));
					fillConvexPoly(image,vertices,4,Scalar(255,255,0));
					//cout << "to dikhana saale" <<endl;
					imshow("IMG",image);
					//cout << maxi << " " << maxj << endl;
					waitKey(10);
					fillConvexPoly(image,vertices,4,Scalar(0,0,0));
				}

				//cout << "he" <<endl;
				//cout << "he" <<endl;
				cout << r*sin(orientation*(3.14/180)) - r*sin((3.14/180)*(orientation-maxj*0.25)) << " " << r*cos(orientation*(3.14/180)) - r*cos((3.14/180)*(orientation-maxj*0.25))<< endl;
				curr_state.y += r*sin(orientation*(3.14/180)) - r*sin((3.14/180)*(orientation-maxj*0.25));
				curr_state.x += r*cos(orientation*(3.14/180)) - r*cos((3.14/180)*(orientation-maxj*0.25));
				curr_state.orientation -= maxj*0.25;
				//cout << curr_state.orientation <<endl;
				curr_state.vel = maxi;
				curr_state.ang_vel = maxj;
				cout << "New position is : " << curr_state.x << " " << curr_state.y <<endl;
				cout << "Angleo : " << atan((end.x-x)/(end.y-y))*(180/3.14) <<endl;
				cout << "Angle : " << curr_state.orientation <<endl;
				cout << "Velocity : " << curr_state.vel <<endl;
				cout << "Angular velocity : " << curr_state.ang_vel <<endl;
				if(image.at<Vec3b>(curr_state.x,curr_state.y)[2]==255) return;
				//return;
			}
		
		}
	private:
		Point start;
		Point end;
		Mat map;
		int length,bredth;
		state curr_state;
		vector<state> path; 
		int acceleration;
		int ang_acceleration;
};

int main(int argc,char** argv) {
	image = imread(argv[1],1);
	
	for(int j=0;j<image.cols;j++){
		image.at<Vec3b>(0,j) = {255,255,255};
		image.at<Vec3b>(image.rows-1,j) = {255,255,255};
	}
	for(int i=0;i<image.rows;i++){
		image.at<Vec3b>(i,image.cols-1) = {255,255,255};
		image.at<Vec3b>(i,0) = {255,255,255};
	}
	Point start(0,0);Point end(0,0);int t1=0,t2=0;
	int i,j;
	for(i=0;i<image.rows;i++){
		for(j=0;j<image.cols;j++){
			if(image.at<Vec3b>(i,j)[0] == 0 && image.at<Vec3b>(i,j)[1] ==255 && image.at<Vec3b>(i,j)[2] == 0){
				start.x += i;start.y+=j;t1++;
			}
			if(image.at<Vec3b>(i,j)[0] == 0 && image.at<Vec3b>(i,j)[1] ==0 && image.at<Vec3b>(i,j)[2] == 255){
				end.x += i;end.y+=j;t2++;
			}
		}
	}
	start.x/=t1;start.y/=t1;end.x/=t2;end.y/=t2;
	cout << start.x << " " << start.y <<endl;
	cout << end.x << " " << end.y <<endl;
	//return 0;
	PathPlanner dvij(start,end,image,5,5);
	dvij.getPath();
	cout << "Hurrah!!!!!!!!!!!!!!!!!!" <<endl;
}