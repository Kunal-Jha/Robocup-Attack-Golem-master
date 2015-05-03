// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

//bin/rione_player -team Ri-one -f conf/formations.conf -c conf/player.conf -num 1


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "sample_player.h"

#include "strategy.h"
#include "field_analyzer.h"

#include "action_chain_holder.h"
#include "sample_field_evaluator.h"

#include "soccer_role.h"

#include "sample_communication.h"
#include "keepaway_communication.h"

#include "bhv_penalty_kick.h"
#include "bhv_set_play.h"
#include "bhv_set_play_kick_in.h"
#include "bhv_set_play_indirect_free_kick.h"
#include "bhv_basic_move.h" 

#include "bhv_custom_before_kick_off.h"
#include "bhv_strict_check_shoot.h"

#include "view_tactical.h"

#include "intention_receive.h"

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/bhv_emergency.h>
#include <rcsc/action/body_go_to_point.h>
 #include <rcsc/action/body_kick_one_step.h>

#include <rcsc/action/body_intercept.h>
#include <rcsc/action/body_smart_kick.h>
#include <rcsc/action/body_kick_one_step.h>
#include <rcsc/action/neck_scan_field.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>
#include <rcsc/action/neck_turn_to_point.h> 
#include <rcsc/action/view_synch.h>
#include <rcsc/action/body_hold_ball.h>
#include <rcsc/action/body_dribble.h>

#include <rcsc/formation/formation.h>
#include <rcsc/action/kick_table.h>
#include <rcsc/player/intercept_table.h>
#include <rcsc/player/say_message_builder.h>
#include <rcsc/player/audio_sensor.h>
#include <rcsc/player/freeform_parser.h>
#include "bhv_chain_action.h"
#include <rcsc/action/body_advance_ball.h>
#include <rcsc/action/body_dribble.h>
#include <rcsc/action/body_hold_ball.h>
#include <rcsc/action/body_pass.h>
#include <rcsc/action/neck_scan_field.h>
#include <rcsc/action/neck_turn_to_low_conf_teammate.h>
#include <rcsc/common/basic_client.h>
#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/common/player_param.h>
#include <rcsc/common/audio_memory.h>
#include <rcsc/common/say_message_parser.h>
// #include <rcsc/common/free_message_parser.h>

#include <rcsc/param/param_map.h>
#include <rcsc/param/cmd_line_parser.h>

#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <vector> 

using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

 */
SamplePlayer::SamplePlayer()
    : PlayerAgent(),
      M_communication(),
      M_field_evaluator( createFieldEvaluator() ),
      M_action_generator( createActionGenerator() )
{
    boost::shared_ptr< AudioMemory > audio_memory( new AudioMemory );

    M_worldmodel.setAudioMemory( audio_memory );

    //
    // set communication message parser
    //
    addSayMessageParser( SayMessageParser::Ptr( new BallMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new PassMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new InterceptMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new GoalieMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new GoalieAndPlayerMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new OffsideLineMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new DefenseLineMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new WaitRequestMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new PassRequestMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new DribbleMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new BallGoalieMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new OnePlayerMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new TwoPlayerMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new ThreePlayerMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new SelfMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new TeammateMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new OpponentMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new BallPlayerMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new StaminaMessageParser( audio_memory ) ) );
    addSayMessageParser( SayMessageParser::Ptr( new RecoveryMessageParser( audio_memory ) ) );

    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 9 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 8 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 7 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 6 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 5 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 4 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 3 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 2 >( audio_memory ) ) );
    // addSayMessageParser( SayMessageParser::Ptr( new FreeMessageParser< 1 >( audio_memory ) ) );

    //
    // set freeform message parser
    //
    setFreeformParser( FreeformParser::Ptr( new FreeformParser( M_worldmodel ) ) );

    //
    // set action generators
    //
    // M_action_generators.push_back( ActionGenerator::Ptr( new PassGenerator() ) );

    //
    // set communication planner
    //
    M_communication = Communication::Ptr( new SampleCommunication() );
}

/*-------------------------------------------------------------------*/
/*!

 */
SamplePlayer::~SamplePlayer()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
bool
SamplePlayer::initImpl( CmdLineParser & cmd_parser )
{
    bool result = PlayerAgent::initImpl( cmd_parser );

    // read additional options
    result &= Strategy::instance().init( cmd_parser );

    rcsc::ParamMap my_params( "Additional options" );
#if 0
    std::string param_file_path = "params";
    param_map.add()
        ( "param-file", "", &param_file_path, "specified parameter file" );
#endif

    cmd_parser.parse( my_params );

    if ( cmd_parser.count( "help" ) > 0 )
    {
        my_params.printHelp( std::cout );
        return false;
    }

    if ( cmd_parser.failed() )
    {
        std::cerr << "player: ***WARNING*** detected unsuppprted options: ";
        cmd_parser.print( std::cerr );
        std::cerr << std::endl;
    }

    if ( ! result )
    {
        return false;
    }

    if ( ! Strategy::instance().read( config().configDir() ) )
    {
        std::cerr << "***ERROR*** Failed to read team strategy." << std::endl;
        return false;
    }

    if ( KickTable::instance().read( config().configDir() + "/kick-table" ) )
    {
        std::cerr << "Loaded the kick table: ["
                  << config().configDir() << "/kick-table]"
                  << std::endl;
    }

    return true;
}

/*-------------------------------------------------------------------*/
/*!
  main decision
  virtual method in super class
*/
void
SamplePlayer::actionImpl()
{
    //
    // update strategy and analyzer
    //
    Strategy::instance().update( world() );
    FieldAnalyzer::instance().update( world() );

    //
    // prepare action chain
    //
    M_field_evaluator = createFieldEvaluator();
    M_action_generator = createActionGenerator();

    ActionChainHolder::instance().setFieldEvaluator( M_field_evaluator );
    ActionChainHolder::instance().setActionGenerator( M_action_generator );

    //
    // special situations (tackle, objects accuracy, intention...)
    //
    const WorldModel & wm = this->world();

    if ( doPreprocess() )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": preprocess done" );
        return;
    }

    //
    // update action chain
    //
    ActionChainHolder::instance().update( world() );

    //
    // create current role
    //
    std::string role_name;
    SoccerRole::Ptr role_ptr;
    {
        role_ptr = Strategy::i().createRole( world().self().unum(), world() );
        role_name = Strategy::i().getRoleName( world().self().unum(), world() );

        if ( ! role_ptr )
        {
            std::cerr << config().teamName() << ": "
                      << world().self().unum()
                      << " Error. Role is not registerd.\nExit ..."
                      << std::endl;
            M_client->setServerAlive( false );
            return;
        }
    }

    //
    // override execute if role accept
    //
    if ( role_ptr->acceptExecution( world() ) )
    {
        if(role_name=="Sample"){
            executeSampleRole(this);
            return;
        }
        role_ptr->execute( this );
        return;
    }

    //
    // play_on mode
    //
    if ( world().gameMode().type() == GameMode::PlayOn )
    {
        if(role_name=="Sample"){
            executeSampleRole(this);
            return;
        }
        role_ptr->execute( this );
        return;
    }

    if ( world().gameMode().type() == GameMode::KickOff_)
    {   
        mpIntransit = false;
        if(wm.self().unum()==10){
            mpIntransit = true;
            mpTarget = wm.ball().pos();
        }
        bool kickable = this->world().self().isKickable();
        if ( this->world().existKickableTeammate()
             && this->world().teammatesFromBall().front()->distFromBall()
             < this->world().ball().distFromSelf() )
        {
            kickable = false;
        }

        if ( kickable )
        {
            //Bhv_BasicMove().execute(this);
            Bhv_BasicOffensiveKick().execute(this);
        //    if(!PassToBestPlayer( this )){
        //        Body_HoldBall().execute( this );
                //doKick( this);
                //Bhv_BasicOffensiveKick().execute(this);
                //PassToBestPlayer(this);   
            //f}                
        }
        else
        {
            Bhv_BasicMove().execute(this);
        }
        return;
    }

    //
    // penalty kick mode
    //
    if ( world().gameMode().isPenaltyKickMode() )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": penalty kick" );
        Bhv_PenaltyKick().execute( this );
        return;
    }

    //
    // other set play mode
    //
    Bhv_SetPlay().execute( this );
    return;

    if(role_name=="Sample"){
        executeSampleRole(this);
        return;
    }
}

int 
SamplePlayer::hasball(PlayerAgent *agent)
{
    const WorldModel & wm = agent->world();
    double mindis = 999;
    int mindisunum;
    for(int i=2; i<=11;i++)
    {
        if(agent->world().ourPlayer(i)!=NULL){
            if(agent->world().ourPlayer(i)->distFromBall() < mindis)
            {
                mindis = agent->world().ourPlayer(i)->distFromBall();
                mindisunum = i;
            }
        }
    }
    return mindisunum ;

} 
    bool
    SamplePlayer::ultaSectorEmpty(PlayerAgent * agent, Vector2D target, double buf_degree){
    Neck_TurnToPoint( target ).execute(agent );
    const WorldModel & wm = agent->world();
    AngleDeg target_angle = (target - wm.self().pos()).dir();
    double target_dis = wm.self().pos().dist(target);
    const Sector2D sector( target,
                            0.0, target_dis ,
                            target_angle - buf_degree,
                            target_angle + buf_degree );
    // opponent check without goalie
    if ( ! wm.existOpponentIn( sector, 10, false ) )
    {
        return true;
    }

    return false;
}

bool
SamplePlayer::amifree(PlayerAgent * agent)
{

    int mindisunum = hasball(agent);
    

    if( ultaSectorEmpty(agent,agent->world().ourPlayer(mindisunum)->pos(),5.0))
    {
        return true;
    }
    else 
    {
        return false;
    }
}
bool 
SamplePlayer::getfree(PlayerAgent *agent)
{
    const WorldModel & wm = agent->world();
    int mindisunum = hasball(agent);
    if(amifree(agent))
    {
        return true;
    }
    else
    {
        const Vector2D origin(0.0,0.0);
        Triangle2D *tr= new Triangle2D(origin,wm.self().pos(),agent->world().ourPlayer(mindisunum)->pos());
        Vector2D bhaag = tr->circumcenter();
        Body_GoToPoint( bhaag, 0.0, ServerParam::i().maxDashPower(), -1, 4, true, 60).execute( agent );
        agent->setNeckAction(new Neck_TurnToBall());
        return true;

    }
    
}



/*--------------------------------------------------------------------------*/
bool
SamplePlayer::triangulators(PlayerAgent * agent){
    const WorldModel & wm = agent->world();
    
    double mindis1 = 999;
    double mindis2 = 999;
    double mindis3 = 999;
    double mindis4 = 999;
    int mindisunum4 =-1;            
    int mindisunum1 = -1;
    int mindisunum2 = -1;
    int mindisunum3 = -1;
    for(int i=2; i<=11; i++){
        if(agent->world().ourPlayer(i)!=NULL){
            if(agent->world().ourPlayer(i)->distFromBall() < mindis4)
            {   
                mindis1 = mindis3;
                mindis3 = mindis2;
                mindis2 = mindis4;
                mindisunum1 = mindisunum3;
                mindisunum3 = mindisunum2;
                mindisunum2 = mindisunum4;
                mindis4 = agent->world().ourPlayer(i)->distFromBall();
                mindisunum4 = i;

            }
            else if(agent->world().ourPlayer(i)->distFromBall() < mindis2)
            {
                mindis1 = mindis3;
                mindis3 = mindis2;
                mindisunum1 = mindisunum3;
                mindisunum3 = mindisunum2;
                mindisunum2 = i;
                mindis2 = agent->world().ourPlayer(i)->distFromBall();
            }
            else if(agent->world().ourPlayer(i)->distFromBall() < mindis3)
            {
                mindis1 = mindis3;
                mindisunum1 = mindisunum3;
                mindisunum3 = i;
                mindis3 = agent->world().ourPlayer(i)->distFromBall();
            }
            else if(agent->world().ourPlayer(i)->distFromBall()<mindis1)
            {
                 mindisunum1 = i;
                mindis1 = agent->world().ourPlayer(i)->distFromBall();   
            }


            }
        
    }
    Vector2D goal(ServerParam::i().pitchHalfLength(),0.0);
    Triangle2D *tr2 = new Triangle2D(agent->world().ourPlayer(mindisunum4)->pos(),goal,wm.ball().pos());
    Vector2D bhaag4 = tr2->centroid();
  Triangle2D *tr= new Triangle2D(agent->world().ourPlayer(mindisunum1)->pos(),agent->world().ourPlayer(mindisunum2)->pos(),agent->world().ourPlayer(mindisunum3)->pos());  
  Vector2D bisector = tr->incenter();
  if(wm.self().unum() == mindisunum4)
  {
    Body_GoToPoint( bhaag4, 0.0, ServerParam::i().maxDashPower(), -1, 4, true, 60).execute( agent );
    agent->setNeckAction(new Neck_TurnToLowConfTeammate());
  return true;

  }
  double slope2 = (bisector.y - agent->world().ourPlayer(mindisunum2)->pos().y)/
                        (bisector.x - agent->world().ourPlayer(mindisunum2)->pos().x);
double slope3 = (bisector.y - agent->world().ourPlayer(mindisunum3)->pos().y)/
                        (bisector.x - agent->world().ourPlayer(mindisunum3)->pos().x);
  if(wm.self().unum() == mindisunum2 || wm.self().unum()==mindisunum3){
   if(wm.self().unum()==mindisunum3)
   {
    double c2= bisector.y-(slope3*bisector.x);
slope3= (-1) *slope3;
  double x_final2 = agent->world().ourPlayer(mindisunum3)->pos().x +20.0;
  double y_final2 =   (slope3*x_final2) + c2;
Vector2D bhaag2 = Vector2D(x_final2,y_final2);
Body_GoToPoint( bhaag2, 0.0, ServerParam::i().maxDashPower(), -1, 4, true, 60).execute( agent );
agent->setNeckAction(new Neck_TurnToLowConfTeammate());
  return true;
   }
   if(wm.self().unum()==mindisunum2)
   {double c1= bisector.y-(slope2*bisector.x);
  slope2= (-1) *slope2;
            
  double x_final1 = agent->world().ourPlayer(mindisunum2)->pos().x +20.0;
  double y_final1 =   (slope2*x_final1) + c1;
Vector2D bhaag1 = Vector2D(x_final1,y_final1);
Body_GoToPoint( bhaag1, 0.0, ServerParam::i().maxDashPower(), -1, 4, true, 60).execute( agent );
agent->setNeckAction(new Neck_TurnToLowConfTeammate());
return true;
   }
return false;
  }
  

              

    
   /* double slope12 = (agent->world().ourPlayer(mindisunum2).y - agent->world().ourPlayer(mindisunum1).y)/
                        (agent->world().ourPlayer(mindisunum2).x - agent->world().ourPlayer(mindisunum1).x);

    double slope23 = (agent->world().ourPlayer(mindisunum3).y - agent->world().ourPlayer(mindisunum2).y)/
                        (agent->world().ourPlayer(mindisunum3).x - agent->world().ourPlayer(mindisunum2).x);
    */
    
    /*double x1 = (double)(agent->world().ourPlayer(mindisunum1)->pos().x);
    double y1 = (double)(agent->world().ourPlayer(mindisunum1)->pos().y);
    double x2 = (double)(agent->world().ourPlayer(mindisunum2)->pos().x);
    double y2 = (double)(agent->world().ourPlayer(mindisunum2)->pos().y);
    double x3 = (double)(agent->world().ourPlayer(mindisunum3)->pos().x);
    double y3 = (double)(agent->world().ourPlayer(mindisunum3)->pos().y);
    double angle1 = acos((pow(DistanceBetweenPoints(x1,y1,x2,y2),2)+pow(DistanceBetweenPoints(x1,y1,x3,y3),2)-
                            pow(DistanceBetweenPoints(x2,y2,x3,y3),2))/
                            (2*DistanceBetweenPoints(x1,y1,x2,y2)*DistanceBetweenPoints(x1,y1,x3,y3)));;
    angle1=angle1/2;
    if(wm.self().unum() == mindisunum2 || wm.self().unum()==mindisunum3)
    {
        if(wm.self().unum()==mindisunum3)
        {
                double slope12 = (agent->world().ourPlayer(mindisunum2)->pos().y - agent->world().ourPlayer(mindisunum1)->pos().y)/
                        (agent->world().ourPlayer(mindisunum2)->pos().x - agent->world().ourPlayer(mindisunum1)->pos().x);
                        
                double x_final =  wm.self().pos().x + 15.0;
                double c =   agent->world().ourPlayer(mindisunum2)->pos().y- slope12*agent->world().ourPlayer(mindisunum2)->pos().x;
                double y_final = slope12*agent->world().ourPlayer(mindisunum2)->pos().x + c;
                Vector2D runtopoint = Vector2D(x_final,y_final);
                 Body_GoToPoint( runtopoint, 0.0, ServerParam::i().maxDashPower(), -1, 4, true, 60).execute( agent );
         }
         if(wm.self().unum()==mindisunum2)
        {
                double slope13 = (agent->world().ourPlayer(mindisunum3)->pos().y - agent->world().ourPlayer(mindisunum1)->pos().y)/
                        (agent->world().ourPlayer(mindisunum3)->pos().x - agent->world().ourPlayer(mindisunum1)->pos().x);
                double x_final =  wm.self().pos().x + 15.0;
                double c =   agent->world().ourPlayer(mindisunum3)->pos().y- slope13*agent->world().ourPlayer(mindisunum3)->pos().x;
                double y_final = slope13*agent->world().ourPlayer(mindisunum3)->pos().x + c;
                Vector2D runtopoint = Vector2D(x_final,y_final);
                 Body_GoToPoint( runtopoint, 0.0, ServerParam::i().maxDashPower(), -1, 4, true, 60).execute( agent );
         }
         
    }
return true;*/

}


bool
SamplePlayer::PassToPlayer( PlayerAgent * agent, Vector2D target_point, int receiver )
{
    if ( ! agent->world().self().isKickable() || receiver < 2)
    {
        return false;
    }

    double first_speed = ServerParam::i().ballSpeedMax()*0.75;
 agent->debugClient().addMessage( "pass" );
    agent->debugClient().setTarget( target_point );

    int kick_step = ( agent->world().gameMode().type() != GameMode::PlayOn
                      && agent->world().gameMode().type() != GameMode::GoalKick_
                      ? 1
                      : 3 );
    if ( ! Body_SmartKick( target_point,
                           first_speed,
                           first_speed * 0.96,
                           kick_step ).execute( agent ) )
    {
        if ( agent->world().gameMode().type() != GameMode::PlayOn
             && agent->world().gameMode().type() != GameMode::GoalKick_ )
        {
            first_speed = std::min( agent->world().self().kickRate() * ServerParam::i().maxPower(),
                                    first_speed );
            Body_KickOneStep( target_point,
                              first_speed
                              ).execute( agent );
        }
        else
        {
            return false;
        }
    }

    if ( agent->config().useCommunication()
         && receiver != Unum_Unknown )
    {
        Vector2D target_buf = target_point - agent->world().self().pos();
        target_buf.setLength( 1.0 );

        agent->addSayMessage( new PassMessage( receiver,
                                               target_point + target_buf,
                                               agent->effector().queuedNextBallPos(),
                                               agent->effector().queuedNextBallVel() ) );
    }

    return true;
}
bool
SamplePlayer::ThroughPass(PlayerAgent *agent)
{

    const WorldModel & wm = agent->world();
    const PlayerPtrCont & opps = wm.opponentsFromSelf();
    const PlayerObject * nearest_opp
        = ( opps.empty()
            ? static_cast< PlayerObject * >( 0 )
            : opps.front() );
    const double nearest_opp_dist = ( nearest_opp
                                      ? nearest_opp->distFromSelf()
                                      : 1000.0 );
    const Vector2D nearest_opp_pos = ( nearest_opp
                                       ? nearest_opp->pos()
                                       : Vector2D( -1000.0, 0.0 ) );
    const Vector2D leftv(0.0,10.0);
    const Vector2D rightv(0.0,-10.0);   
    const Vector2D left_point = leftv + nearest_opp_pos;
    const Vector2D right_point = rightv + nearest_opp_pos;
    if(left_point.absY() > ServerParam::i().pitchHalfWidth() - 3)
    {
        return false;
    }

    if(IsSectorEmpty(agent,left_point,25.0) && ClosestPlayerToVector(agent,left_point) != -1)
    {
        int temp = ClosestPlayerToVector(agent, left_point); 
        PassToPlayer(agent,left_point,ClosestPlayerToVector(agent,left_point));
        agent->setNeckAction(new Neck_TurnToBall());
        return true;
    }
    if(right_point.absY() > ServerParam::i().pitchHalfWidth() - 3)
    {
        return false;
    }
    
    if(IsSectorEmpty(agent,right_point,25.0) && ClosestPlayerToVector(agent,right_point) != -1)
    {
        PassToPlayer(agent,right_point,ClosestPlayerToVector(agent,right_point));
            agent->setNeckAction(new Neck_TurnToBall());
        return true;
    }
    return false;
}
int
SamplePlayer::ClosestPlayerToVector(PlayerAgent *agent, Vector2D vector)
{
    int ans=-1;
    const WorldModel & wm = agent->world();
    const PlayerPtrCont & opps = wm.opponentsFromSelf();
    int player =-1;
    int minimum = 999;
            const PlayerPtrCont::const_iterator opps_end = opps.end();
            for ( PlayerPtrCont::const_iterator it = opps.begin();
                  it != opps_end;
                  ++it )
            {
                if ( (*it)->pos().dist( vector ) < minimum )
                {
                    minimum = (*it)->pos().dist( vector );
                    player = (*it)->unum();
                }
            }


    for(int i=2;i<=11;i++)
    {
        if(agent->world().ourPlayer(i) != NULL && i !=  wm.self().unum())
        {

        if(
            agent->world().ourPlayer(i)->pos().dist(vector) <
             minimum
            )
        {         
            ans = i;
        }

    }
    }
    return ans;
}
bool 
SamplePlayer::converge(PlayerAgent *agent)
{
    const WorldModel & wm = agent->world();
    int mindisnum = hasball(this);
    if(wm.self().pos().x > 0 && wm.self().unum() != mindisnum)
    {
        const Vector2D origin(0.0,0.0);
        Triangle2D *tr= new Triangle2D(origin,wm.self().pos(),agent->world().ourPlayer(mindisnum)->pos());
        Vector2D bhaag5= tr->circumcenter();
        Body_GoToPoint( bhaag5, 0.0, ServerParam::i().maxDashPower(), -1, 4, true, 60).execute( agent );
        agent->setNeckAction(new Neck_TurnToBall());
    }
  
    return true;
}

bool 
SamplePlayer::ApnaDribble (PlayerAgent * agent)
{
    const WorldModel & wm = agent->world();
    const PlayerPtrCont & opps = wm.opponentsFromSelf();
    const PlayerObject * nearest_opp
        = ( opps.empty()
            ? static_cast< PlayerObject * >( 0 )
            : opps.front() );
    const double nearest_opp_dist = ( nearest_opp
                                      ? nearest_opp->distFromSelf()
                                      : 1000.0 );
    const Vector2D nearest_opp_pos = ( nearest_opp
                                       ? nearest_opp->pos()
                                       : Vector2D( -1000.0, 0.0 ) );
    const Vector2D drib_target( 10.0, wm.self().pos().absY() );
    const Vector2D body_dir_drib_target
            = wm.self().pos()
            + Vector2D::polar2vector(5.0, wm.self().body());
            
    const Vector2D leftv(0.0,10.0);
    const Vector2D rightv(0.0,-10.0);     
    const Vector2D left_point = leftv + nearest_opp_pos;
    const Vector2D right_point = rightv + nearest_opp_pos; 
    if(doShoot())
    {
        return true;
    }
    if ( nearest_opp_dist < 2.0 && wm.self().pos().x >  ServerParam::i().theirPenaltyAreaLineX() && wm.self().pos().absY() <20)
         
{
    bool kickable = agent->world().self().isKickable();
    const Vector2D body_dir_drib_target
            = wm.self().pos()
            + Vector2D::polar2vector(2.0, wm.self().body());
                
          const Vector2D leftvd(0.0,2.0);
    const Vector2D rightvd(0.0,-2.0);     
    const Vector2D left_pointd = leftvd + nearest_opp_pos;
    const Vector2D right_pointd = rightvd + nearest_opp_pos;      
    if(IsSectorEmpty(agent, body_dir_drib_target, 2.0))
            {
                if(wm.self().pos().y<= 0.0)
                {
                     Body_Dribble( left_pointd,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true; 
                }
                else
                {
                    Body_Dribble( right_pointd,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true; 
                }
            }
             else if(IsSectorEmpty(agent,left_pointd, 2.0))
              {
                 Body_Dribble( left_pointd,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true; 
                  
              }
              else if(IsSectorEmpty(agent,right_pointd, 2.0))
              {
                 Body_Dribble( right_pointd,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true; 
                  
              }
              else
              {
                  if ( Body_Pass().execute( agent ) )
                {
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true;
                }
                
              }
                
                return true;
         }

     
    if(wm.self().pos().x > 0 && wm.self().pos().absX() < ServerParam::i().theirPenaltyAreaLineX()) 
    {               
    if(ThroughPass(agent))

    {
        agent->setNeckAction(new Neck_TurnToLowConfTeammate());
        return true;
    }}
    
     if ( nearest_opp_dist < 3.0 )
    {
        if ( Body_Pass().execute( agent ) )
        {
            
            agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
            return true;
        }
    }
     if(wm.self().pos().x >= ServerParam::i().theirPenaltyAreaLineX())
        {
            double final_y;
            if(wm.self().pos().y<0)
                final_y = -10;
            if(wm.self().pos().y>=0)
                final_y = 10;
            const Vector2D drib_target1(ServerParam::i().pitchHalfLength()-5,final_y);
          Body_Dribble( drib_target1,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
               
    

            
                return true;
        }     
   
     if ( nearest_opp_dist < 5.0 &&
         nearest_opp_dist > ( ServerParam::i().tackleDist()
                                 + ServerParam::i().defaultPlayerSpeedMax() * 1.5 )
         && wm.self().body().abs() < 70.0 )
         {
          //   std::cout<< " 22222222222222222222222222222222222222222" << std::endl;
                
             if(IsSectorEmpty(agent, body_dir_drib_target, 20.0))
            {
                if(wm.self().pos().y<= 0.0)
                {
                     Body_Dribble( left_point,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true; 
                }
                else
                {
                    Body_Dribble( right_point,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true; 
                }
            }
             else if(IsSectorEmpty(agent,left_point, 10.0))
              {
                 Body_Dribble( left_point,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true; 
                  
              }
              else if(IsSectorEmpty(agent,right_point, 10.0))
              {
                 Body_Dribble( right_point,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true; 
                  
              }
              else
              {
                  if ( Body_Pass().execute( agent ) )
                {
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true;
                }
                
              }
                
                return true;
         }
    if ( nearest_opp_dist > 3.0 )
    {  
       if(wm.self().pos().y < 0.0)
        {
            
          const Vector2D drib_target(ServerParam::i().pitchHalfLength(),-(ServerParam::i().pitchHalfWidth()-5));
          Body_Dribble( drib_target,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
            
            
                return true;

        }
        if(wm.self().pos().y > 0.0)
        {
            const Vector2D drib_target(ServerParam::i().pitchHalfLength(),(ServerParam::i().pitchHalfWidth()-5));
          Body_Dribble( drib_target,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
               
    
                
                return true;

        }
    }  
    
    {
     if ( Body_Pass().execute( agent ) )
                {
            
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true;
                }
    }
    return true; 
                               
}
bool
SamplePlayer::IsSectorEmpty(PlayerAgent * agent, Vector2D target, double buf_degree){
    Neck_TurnToPoint( target ).execute(agent );
    const WorldModel & wm = agent->world();
    AngleDeg target_angle = (target - wm.self().pos()).dir();
    double target_dis = wm.self().pos().dist(target);
    const Sector2D sector( wm.self().pos(),
                            0.5, target_dis + 4,
                            target_angle - buf_degree,
                            target_angle + buf_degree );
    // opponent check without goalie
    if ( ! wm.existOpponentIn( sector, 10, false ) )
    {
        return true;
    }

    return false;
}



int
SamplePlayer::ClosestPlayerToBall(PlayerAgent * agent){
    double mindis = 999;
    int mindisunum = -1;
    for(int i=2; i<=11; i++){
        if(agent->world().ourPlayer(i)!=NULL){
            if(agent->world().ourPlayer(i)->distFromBall() < mindis){
                mindis = agent->world().ourPlayer(i)->distFromBall();
                mindisunum = i;
            }
        }
    }
    return mindisunum;
}


//main function that will be used.

bool
SamplePlayer::executeSampleRole( PlayerAgent * agent )
{
     const WorldModel & wm = agent->world();
         
    if(agent->config().teamName()=="opp"){
        Body_GoToPoint( Vector2D(-50,0), 0.0, ServerParam::i().maxDashPower(), -1, 4, true, 60).execute( agent );
        return true;
    }

    //Setting up of different flags.

    bool kickable = agent->world().self().isKickable();
    const Vector2D body_dir_drib_target
            = wm.self().pos()
            + Vector2D::polar2vector(5000.0, wm.self().body());
    
    // If there is a teammate who can kick the ball and who is closer to the ball than I am.
    if ( agent->world().existKickableTeammate()
         && agent->world().teammatesFromBall().front()->distFromBall()
         < agent->world().ball().distFromSelf() )
    {
        kickable = false;
    }


    if(agent->world().existKickableTeammate()){
        //passHeard = false;
        Opponenthasball = false;
    }
    else{};
        //Opponenthasball = true;
    

    if(agent->world().existKickableOpponent()){
        Opponenthasball = true;
    }

    //----------XX------------//
    //If you have taken attack, you will have comment out the attach functions are replace
    //them with your own, similarly if you have taken defense, you need to comment out the existing
    //defence function and replace it with your own.
    //------------xx------------//


    //ATTACK STARTS HERE
    // I have the ball, what to do?
    if ( kickable && !Opponenthasball)
    {
        //SampleDribble(this);
        ApnaDribble(this);
        return true;
        //doKick( this);
       /* if(IsSectorEmpty(agent, body_dir_drib_target, 25.0))
        {
               // std::cout << "maja aa gaya------------------------------------------"<<std::endl;
            Body_Dribble( body_dir_drib_target,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        }
        else
        {
        * 
            Body_Pass().execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        }
     
     return true;
     */                  
    }

    //This is for off the ball movement which attacking, where to go for passes etc.
    else if (!kickable && !Opponenthasball)
    {
        if(wm.ball().pos().x > ServerParam::i().theirPenaltyAreaLineX())
        {
        converge(this);
        }  
        triangulators(this);
        // getfree(this);
        doMove(this);
        return true;
    } 
    //ATTACK ENDS HERE
    //--------XX----------XX--------//
    // DEFENCE STARTS HERE
    //The defense call
    else
    {
        //Uncomment this for defense.
        //If I can kick the ball, but opponent has it. Common ball.
        if (kickable && Opponenthasball){
            if ( Bhv_ChainAction().execute( agent ) )
            {
                dlog.addText( Logger::TEAM,
                      __FILE__": (execute) do chain action" );
            agent->debugClient().addMessage( "ChainAction" );
            return true;
            }   

            Bhv_BasicOffensiveKick().execute( agent );
            return true;

            }

        // I don't have the ball, opponent has it, off the ball movement while defending.
        //falling back etc.     
        else if (!kickable && Opponenthasball){
            Bhv_BasicMove().execute(agent);
        }
        return true;
    };

    //DEFENSE ENDS HERE.

    return true;
}

/*-------------------------------------------------------------------*/
/*!

*/
void
SamplePlayer::doKick( PlayerAgent * agent )
{
    
    if ( Bhv_ChainAction().execute( agent ) )
    {
        return;
    }

    Bhv_BasicOffensiveKick().execute( agent );
}



/*-------------------------------------------------------------------*/
/*!

*/
void
SamplePlayer::doMove( PlayerAgent * agent )
{
        Bhv_BasicMove().execute(agent);
        return;
}

bool
SamplePlayer::BasicMove(PlayerAgent * agent){
    const WorldModel & wm = agent->world();
    
    //-----------------------------------------------
    // tackle
    
    if ( Bhv_BasicTackle( 0.8, 80.0 ).execute( agent ) )
    {   
        return true;
    }

    /*--------------------------------------------------------*/
    // chase ball
    const int self_min = wm.interceptTable()->selfReachCycle();
    const int mate_min = wm.interceptTable()->teammateReachCycle();
    const int opp_min = wm.interceptTable()->opponentReachCycle();

    //Intercept
    if ( ! wm.existKickableTeammate()
         && ( self_min <= 3
              || ( self_min <= mate_min
                   && self_min < opp_min + 3 )
              )
         )
    {
        std::cout<<"body intercept called for player - "<<wm.self().unum()<<std::endl;

    
        dlog.addText( Logger::TEAM,
                      __FILE__": intercept");
        Body_Intercept().execute( agent );
        agent->setNeckAction( new Neck_OffensiveInterceptNeck() );
        return true;
    }
    
    //Check if ball has been passed
    /*
    if(wm.existKickableTeammate()){
        int CurrentBH = GetBHUnum(agent);
        if(CurrentBH!=LastBH && CurrentBH!=-1){
            FoundNewBH = true;
            LastBH = CurrentBH;
        }
        else
            FoundNewBH = false;

        //Opponenthasball=false;
    }
    */
    
    const Vector2D target_point = Vector2D(0,0);
    const double dash_power = ServerParam::i().maxDashPower();

    double dist_thr = wm.ball().distFromSelf() * 0.1;
    if ( dist_thr < 1.0 ) dist_thr = 1.0;

    

    if ( wm.existKickableOpponent()
         && wm.ball().distFromSelf() < 18.0 )
    {
        agent->setNeckAction( new Neck_TurnToBall() );
        return true;
    }
    else
    {
        agent->setNeckAction( new Neck_TurnToBallOrScan() );
    }

    return true;
}






//centreback







bool
SamplePlayer::isKickable(PlayerAgent * agent, int unum){
    const WorldModel & wm = agent->world();
    if(wm.ourPlayer(unum)!=NULL){
        if(wm.ourPlayer(unum)->distFromBall() < ServerParam::i().defaultKickableArea()){
            return true;
        }
        return false;
    }
    else{
        return false;
    }
}

bool 
SamplePlayer::AreSamePoints(Vector2D A, Vector2D B, double buffer){
    //Check if and b are the same points +/- buffer
    if(A.dist(B)<buffer)
        return true;
    return false;
}

double 
SamplePlayer::abs(double d){
    if (d>0.00)
        return d;
    else
        return d*(-1.00);
}


/*
SamplePlayer::AreSameNos(double A, double B, double buffer){
    if( abs(A-B) < buffer)
        return true;
    return false;
}


*/
double 
SamplePlayer::DistanceBetweenPoints(double x1,double y1,double x2,double y2){
 
        double distance = sqrt(pow((x1-x2),2)+pow((y1-y2),2));

        return distance;
    }



    /*double AngleBetweenPoints(double x1,double y1,double x2,double y2,double x3,double y3){

        double angle = acos((pow(DistanceBetweenPoints(x1,y1,x2,y2),2)+pow(DistanceBetweenPoints(x1,y1,x3,y3),2)-pow(DistanceBetweenPoints(x2,y2,x3,y3),2))/(2*DistanceBetweenPoints(x1,y1,x2,y2)*DistanceBetweenPoints(x1,y1,x3,y3)));
        
        return angle;
    }
*/
    static bool compare_first(const std::pair<double,double>& i, const std::pair<double,double>& j)
    {
        return i.first > j.first;
    }


    static bool compare_second(const std::pair<double,double>& i, const std::pair<double,double>& j)
    {
        return i.second > j.second;
    }

    double slope(double x1,double y1,double x2,double y2){

        double slope_of_line = (y2-y1)/(x2-x1);

        return slope_of_line;
    }

    double constant(double x1,double y1,double slope_of_line){

        return (y1-slope_of_line*x1);
    }

    double bisector_line_const(double c1,double c2,double a1,double a2){


        return ((c2-c1)*sqrt((pow(a2,2)+1)*(pow(a1,2)+1)));
    }

    double bisector_line_x_const(double a1,double a2){

        return ((a2*sqrt(pow(a1,2)+1))-(a1*sqrt(pow(a2,2)+1)));
    }

    double bisector_line_y_const(double a1,double a2){


        return (sqrt(pow(a2,2)+1)-sqrt(pow(a1,2)+1));
    }

    double intersecting_point_x(double c1, double c2, double a, double b){

        double x = ((c2*b - c1*a)/((a*a)+(b*b)));
        
        return x;
    }



    double intersecting_point_y(double c1, double c2, double a, double b){

        double y = ((c2*a + c1*b)/((a*a)+(b*b)));
        
        return y;
    } 

    double angle_between_two_lines(const Line2D & line1,const Line2D & line2 ){

        double theta_1 = atan(line1.getB()/line1.getA()); 
    
        double theta_2 = atan(line2.getB()/line2.getA());

        return (theta_2-theta_1);
    }


    static
    Line2D angle_bisectorof( const Vector2D & origin,
                           const AngleDeg & left,
                           const AngleDeg & right )
      {
          return Line2D( origin, AngleDeg::bisect( left, right ) );
      }





/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleActionStart()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleActionEnd()
{
    if ( world().self().posValid() )
    {
#if 0
        const ServerParam & SP = ServerParam::i();
        //
        // inside of pitch
        //

        // top,lower
        debugClient().addLine( Vector2D( world().ourOffenseLineX(),
                                         -SP.pitchHalfWidth() ),
                               Vector2D( world().ourOffenseLineX(),
                                         -SP.pitchHalfWidth() + 3.0 ) );
        // top,lower
        debugClient().addLine( Vector2D( world().ourDefenseLineX(),
                                         -SP.pitchHalfWidth() ),
                               Vector2D( world().ourDefenseLineX(),
                                         -SP.pitchHalfWidth() + 3.0 ) );

        // bottom,upper
        debugClient().addLine( Vector2D( world().theirOffenseLineX(),
                                         +SP.pitchHalfWidth() - 3.0 ),
                               Vector2D( world().theirOffenseLineX(),
                                         +SP.pitchHalfWidth() ) );
        //
        debugClient().addLine( Vector2D( world().offsideLineX(),
                                         world().self().pos().y - 15.0 ),
                               Vector2D( world().offsideLineX(),
                                         world(Opponenthasball
Opponenthasball
Opponenthasball
Opponenthasball
Opponenthasball
Opponenthasball).self().pos().y + 15.0 ) );

        // outside of pitch

        // top,upper
        debugClient().addLine( Vector2D( world().ourOffensePlayerLineX(),
                                         -SP.pitchHalfWidth() - 3.0 ),
                               Vector2D( world().ourOffensePlayerLineX(),
                                         -SP.pitchHalfWidth() ) );
        // top,upper
        debugClient().addLine( Vector2D( world().ourDefensePlayerLineX(),
                                         -SP.pitchHalfWidth() - 3.0 ),
                               Vector2D( world().ourDefensePlayerLineX(),
                                         -SP.pitchHalfWidth() ) );
        // bottom,lower
        debugClient().addLine( Vector2D( world(Opponenthasball
O).theirOffensePlayerLineX(),
                                         +SP.pitchHalfWidth() ),
                               Vector2D( world().theirOffensePlayerLineX(),
                                         +SP.pitchHalfWidth() + 3.0 ) );
        // bottom,lower
        debugClient().addLine( Vector2D( world().theirDefensePlayerLineX(),
                                         +SP.pitchHalfWidth() ),
                               Vector2D( world().theirDefensePlayerLineX(),
                                         +SP.pitchHalfWidth() + 3.0 ) );
#else
        // top,lower
        debugClient().addLine( Vector2D( world().ourDefenseLineX(),
                                         world().self().pos().y - 2.0 ),
                               Vector2D( world().ourDefenseLineX(),
                                         world().self().pos().y + 2.0 ) );

        //
        debugClient().addLine( Vector2D( world().offsideLineX(),
                                         world().self().pos().y - 15.0 ),
                               Vector2D( world().offsideLineX(),
                                         world().self().pos().y + 15.0 ) );
#endif
    }

    //
    // ball position & velocity
    //
    dlog.addText( Logger::WORLD,
                  "WM: BALL pos=(%lf, %lf), vel=(%lf, %lf, r=%lf, ang=%lf)",
                  world().ball().pos().x,
                  world().ball().pos().y,
                  world().ball().vel().x,
                  world().ball().vel().y,
                  world().ball().vel().r(),
                  world().ball().vel().th().degree() );


    dlog.addText( Logger::WORLD,
                  "WM: SELF move=(%lf, %lf, r=%lf, th=%lf)",
                  world().self().lastMove().x,
                  world().self().lastMove().y,
                  world().self().lastMove().r(),
                  world().self().lastMove().th().degree() );

    Vector2D diff = world().ball().rpos() - world().ball().rposPrev();
    dlog.addText( Logger::WORLD,
                  "WM: BALL rpos=(%lf %lf) prev_rpos=(%lf %lf) diff=(%lf %lf)",
                  world().ball().rpos().x,
                  world().ball().rpos().y,
                  world().ball().rposPrev().x,
                  world().ball().rposPrev().y,
                  diff.x,
                  diff.y );

    Vector2D ball_move = diff + world().self().lastMove();
    Vector2D diff_vel = ball_move * ServerParam::i().ballDecay();
    dlog.addText( Logger::WORLD,
                  "---> ball_move=(%lf %lf) vel=(%lf, %lf, r=%lf, th=%lf)",
                  ball_move.x,
                  ball_move.y,
                  diff_vel.x,
                  diff_vel.y,
                  diff_vel.r(),
                  diff_vel.th().degree() );
}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleServerParam()
{
    if ( KickTable::instance().createTables() )
    {
        std::cerr << world().teamName() << ' '
                  << world().self().unum() << ": "
                  << " KickTable created."
                  << std::endl;
    }
    else
    {
        std::cerr << world().teamName() << ' '
                  << world().self().unum() << ": "
                  << " KickTable failed..."
                  << std::endl;
        M_client->setServerAlive( false );
    }


    if ( ServerParam::i().keepawayMode() )
    {
        std::cerr << "set Keepaway mode communication." << std::endl;
        M_communication = Communication::Ptr( new KeepawayCommunication() );
    }
}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handlePlayerParam()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handlePlayerType()
{

}

/*-------------------------------------------------------------------*/
/*!
  communication decision.
  virtual method in super class
*/
void
SamplePlayer::communicationImpl()
{
    if ( M_communication )
    {
        M_communication->execute( this );
    }
}

/*-------------------------------------------------------------------*/
/*!
*/
bool
SamplePlayer::doPreprocess()
{
    // check tackle expires
    // check self position accuracy
    // ball search
    // check queued intention
    // check simultaneous kick

    const WorldModel & wm = this->world();

    dlog.addText( Logger::TEAM,
                  __FILE__": (doPreProcess)" );

    //
    // freezed by tackle effect
    //
        if ( wm.self().isFrozen() )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": tackle wait. expires= %d",
                      wm.self().tackleExpires() );
        // face neck to ball
        this->setViewAction( new View_Tactical() );
        this->setNeckAction( new Neck_TurnToBallOrScan() );
        return true;
    }
    

    //
    // BeforeKickOff or AfterGoal. jump to the initial position
    //
    if ( wm.gameMode().type() == GameMode::BeforeKickOff
         || wm.gameMode().type() == GameMode::AfterGoal_ )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": before_kick_off" );
        Vector2D move_point =  Strategy::i().getPosition( wm.self().unum() );
        Bhv_CustomBeforeKickOff( move_point ).execute( this );
        this->setViewAction( new View_Tactical() );
        return true;
    }

    //
    // self localization error
    //
    
    if ( ! wm.self().posValid() )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": invalid my pos" );
        Bhv_Emergency().execute( this ); // includes change view
        return true;
    }
    

    //
    // ball localization error
    //
    
    const int count_thr = ( wm.self().goalie()
                            ? 10
                            : 5 );
    if ( wm.ball().posCount() > count_thr
         || ( wm.gameMode().type() != GameMode::PlayOn
              && wm.ball().seenPosCount() > count_thr + 10 ) )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": search ball" );
        this->setViewAction( new View_Tactical() );
        Bhv_NeckBodyToBall().execute( this );
        return true;
    }
    

    //
    // set default change view
    //

    this->setViewAction( new View_Tactical() );

    //
    // check shoot chance
    //
    
    if ( doShoot() )
    {
        std::cout<<"doShoot"<<std::endl;
        std::cout<<"*******************************************************************************"<<std::endl;

        return true;
    }
    

    //
    // check queued action
    //
    
    if ( this->doIntention() )
    {   
       /* std::cout<<"doIntention------------------------------------------------------------"<<std::endl;
        std::cout<<"*******************************************************************************"<<std::endl;
        */dlog.addText( Logger::TEAM,
                      __FILE__": do queued intention" );
        return true;
    }
    
    //
    // check simultaneous kick
    //
    
    if ( doForceKick() )
    {   
        std::cout<<"doForceKick--------------------------------------------------------------"<<std::endl;
        std::cout<<"*******************************************************************************"<<std::endl;
        return true;
    }
    

    //
    // check pass message
    //
    
    if ( doHeardPassReceive() )
    {
        std::cout<<"doHeardPassReceive------------------------------------------------------"<<std::endl;
        std::cout<<"*******************************************************************************"<<std::endl;
        return true;
    }
    

    return false;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
SamplePlayer::doShoot()
{
    const WorldModel & wm = this->world();

    if ( wm.gameMode().type() != GameMode::IndFreeKick_
         && wm.time().stopped() == 0
         && wm.self().isKickable()
         && Bhv_StrictCheckShoot().execute( this ) )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": shooted" );

        // reset intention
        this->setIntention( static_cast< SoccerIntention * >( 0 ) );
        return true;
    }

    return false;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
SamplePlayer::doForceKick()
{
    const WorldModel & wm = this->world();

    if ( wm.gameMode().type() == GameMode::PlayOn
         && ! wm.self().goalie()
         && wm.self().isKickable()
         && wm.existKickableOpponent() )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": simultaneous kick" );
        this->debugClient().addMessage( "SimultaneousKick" );
        Vector2D goal_pos( ServerParam::i().pitchHalfLength(), 0.0 );

        if ( wm.self().pos().x > 36.0
             && wm.self().pos().absY() > 10.0 )
        {
            goal_pos.x = 45.0;
            dlog.addText( Logger::TEAM,
                          __FILE__": simultaneous kick cross type" );
        }
        Body_KickOneStep( goal_pos,
                          ServerParam::i().ballSpeedMax()
                          ).execute( this );
        this->setNeckAction( new Neck_ScanField() );
        return true;
    }

    return false;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
SamplePlayer::doHeardPassReceive()
{
    const WorldModel & wm = this->world();

    if ( wm.audioMemory().passTime() != wm.time()
         || wm.audioMemory().pass().empty()
         || wm.audioMemory().pass().front().receiver_ != wm.self().unum() )
    {
        return false;
    }

    //passHeard = true;

    //Vector2D closestOppPos = wm.getOpponentNearestToSelf(5, false)->pos();

    //closestOppDis = static_cast<int>(wm.self().pos().dist(closestOppPos));


    int self_min = wm.interceptTable()->selfReachCycle();
    Vector2D intercept_pos = wm.ball().inertiaPoint( self_min );
    Vector2D heard_pos = wm.audioMemory().pass().front().receive_pos_;

    dlog.addText( Logger::TEAM,
                  __FILE__":  (doHeardPassReceive) heard_pos(%.2f %.2f) intercept_pos(%.2f %.2f)",
                  heard_pos.x, heard_pos.y,
                  intercept_pos.x, intercept_pos.y );

    if ( ! wm.existKickableTeammate()
         && wm.ball().posCount() <= 1
         && wm.ball().velCount() <= 1
         && self_min < 20
         //&& intercept_pos.dist( heard_pos ) < 3.0 ) //5.0 )
         )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": (doHeardPassReceive) intercept cycle=%d. intercept",
                      self_min );
        this->debugClient().addMessage( "Comm:Receive:Intercept" );
        Body_Intercept().execute( this );
        this->setNeckAction( new Neck_TurnToBall() );
    }
    else
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": (doHeardPassReceive) intercept cycle=%d. go to receive point",
                      self_min );
        this->debugClient().setTarget( heard_pos );
        this->debugClient().addMessage( "Comm:Receive:GoTo" );
        Body_GoToPoint( heard_pos,
                    0.5,
                        ServerParam::i().maxDashPower()
                        ).execute( this );
        this->setNeckAction( new Neck_TurnToBall() );
    }

    this->setIntention( new IntentionReceive( heard_pos,
                                              ServerParam::i().maxDashPower(),
                                              0.9,
                                              5,
                                              wm.time() ) );

    return true;
}

/*-------------------------------------------------------------------*/
/*!

*/
FieldEvaluator::ConstPtr
SamplePlayer::getFieldEvaluator() const
{
    return M_field_evaluator;
}

/*-------------------------------------------------------------------*/
/*!

*/
FieldEvaluator::ConstPtr
SamplePlayer::createFieldEvaluator() const
{
    return FieldEvaluator::ConstPtr( new SampleFieldEvaluator );
}


/*-------------------------------------------------------------------*/
/*!
*/
#include "actgen_cross.h"
#include "actgen_direct_pass.h"
#include "actgen_self_pass.h"
#include "actgen_strict_check_pass.h"
#include "actgen_short_dribble.h"
#include "actgen_simple_dribble.h"
#include "actgen_shoot.h"
#include "actgen_action_chain_length_filter.h"

ActionGenerator::ConstPtr
SamplePlayer::createActionGenerator() const
{
    CompositeActionGenerator * g = new CompositeActionGenerator();

    //
    // shoot
    //
    g->addGenerator( new ActGen_RangeActionChainLengthFilter
                     ( new ActGen_Shoot(),
                       2, ActGen_RangeActionChainLengthFilter::MAX ) );

    //
    // strict check pass
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_StrictCheckPass(), 1 ) );

    //
    // cross
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_Cross(), 1 ) );

    //
    // direct pass
    //
    // g->addGenerator( new ActGen_RangeActionChainLengthFilter
    //                  ( new ActGen_DirectPass(),
    //                    2, ActGen_RangeActionChainLengthFilter::MAX ) );

    //
    // short dribble
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_ShortDribble(), 1 ) );

//std::cout << "maja aa gaya---------------------------------after dribble--"<<std::endl;
    //
    // self pass (long dribble)
    //
    g->addGenerator( new ActGen_MaxActionChainLengthFilter
                     ( new ActGen_SelfPass(), 1 ) );

    //
    // simple dribble
    //
    // g->addGenerator( new ActGen_RangeActionChainLengthFilter
    //                  ( new ActGen_SimpleDribble(),
    //                    2, ActGen_RangeActionChainLengthFilter::MAX ) );

    return ActionGenerator::ConstPtr( g );
}

