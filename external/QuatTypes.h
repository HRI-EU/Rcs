/******************************************************************************

  This code comes from
  Graphics Gems IV, Paul Heckbert (editor), Academic Press, 1994,
  ISBN: 0123361559 (Mac: 0123361567).

  License (see e.g. http://www.realtimerendering.com/resources/GraphicsGems):

  EULA: The Graphics Gems code is copyright-protected. In other words, you
        cannot claim the text of the code as your own and resell it. Using the
        code is permitted in any program, product, or library, non-commercial
        or commercial. Giving credit is not required, though is a nice gesture.
        The code comes as-is, and if there are any flaws or problems with any
        Gems code, nobody involved with Gems - authors, editors, publishers,
        or webmasters - are to be held responsible. Basically, don't be a jerk,
        and remember that anything free comes with no guarantee.

******************************************************************************/

#ifndef _H_QuatTypes
#define _H_QuatTypes
/*** Definitions ***/
typedef struct
{
  double x, y, z, w;
} ShoemakeQuat; /* Quaternion */
enum QuatPart {X, Y, Z, W};
typedef double HMatrix[4][4]; /* Right-handed, for column vectors */
typedef ShoemakeQuat EulerAngles;    /* (x,y,z)=ang 1,2,3, w=order code  */
#endif
