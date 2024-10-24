bool qWhatDoesThis::TestPoint( const q3Transform& tx, const q3Vec3& p ) const
{
	q3Transform world = q3Mul( tx, local );
	q3Vec3 p0 = q3MulT( world, p );

	for ( int i = 0; i < 3; ++i )
	{
		r32 d = p0[ i ];
		r32 ei = e[ i ];

		if ( d > ei || d < -ei )
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------
bool qWhatDoesThis::Raycast( const q3Transform& tx, q3RaycastData* raycast ) const
{
	q3Transform world = q3Mul( tx, local );
	q3Vec3 d = q3MulT( world.rotation, raycast->dir );
	q3Vec3 p = q3MulT( world, raycast->start );
	const r32 epsilon = r32( 1.0e-8 );
	r32 tmin = 0;
	r32 tmax = raycast->t;

	
	r32 t0;
	r32 t1;
	q3Vec3 n0;

	for ( int i = 0; i < 3; ++i )
	{
		if ( q3Abs( d[ i ] ) < epsilon )
		{
			if ( p[ i ] < -e[ i ] || p[ i ] > e[ i ] )
			{
				return false;
			}
		}

		else
		{
			r32 d0 = r32( 1.0 ) / d[ i ];
			r32 s = q3Sign( d[ i ] );
			r32 ei = e[ i ] * s;
			q3Vec3 n( 0, 0, 0 );
			n[ i ] = -s;

			t0 = -(ei + p[ i ]) * d0;
			t1 = (ei - p[ i ]) * d0;

			if ( t0 > tmin )
			{
				n0 = n;
				tmin = t0;
			}

			tmax = q3Min( tmax, t1 );

			if ( tmin > tmax )
			{
				return false;
			}
		}
	}

	raycast->normal = q3Mul( world.rotation, n0 ); 
	raycast->toi = tmin;

	return true;
}

//--------------------------------------------------------------------------------------------------
void qWhatDoesThis::ComputeAABB( const q3Transform& tx, q3AABB* aabb ) const
{
	q3Transform world = q3Mul( tx, local );

	q3Vec3 v[ 8 ] = {
		q3Vec3( -e.x, -e.y, -e.z ),
		q3Vec3( -e.x, -e.y,  e.z ),
		q3Vec3( -e.x,  e.y, -e.z ),
		q3Vec3( -e.x,  e.y,  e.z ),
		q3Vec3(  e.x, -e.y, -e.z ),
		q3Vec3(  e.x, -e.y,  e.z ),
		q3Vec3(  e.x,  e.y, -e.z ),
		q3Vec3(  e.x,  e.y,  e.z )
	};

	for ( i32 i = 0; i < 8; ++i )
		v[ i ] = q3Mul( world, v[ i ] );

	q3Vec3 min( Q3_R32_MAX, Q3_R32_MAX, Q3_R32_MAX );
	q3Vec3 max( -Q3_R32_MAX, -Q3_R32_MAX, -Q3_R32_MAX );

	for ( i32 i = 0; i < 8; ++i )
	{
		min = q3Min( min, v[ i ] );
		max = q3Max( max, v[ i ] );
	}

	aabb->min = min;
	aabb->max = max;
}

//--------------------------------------------------------------------------------------------------
void qWhatDoesThis::ComputeMass( q3MassData* md ) const
{
	r32 ex2 = r32( 4.0 ) * e.x * e.x;
	r32 ey2 = r32( 4.0 ) * e.y * e.y;
	r32 ez2 = r32( 4.0 ) * e.z * e.z;
	r32 mass = r32( 8.0 ) * e.x * e.y * e.z * density;
	r32 x = r32( 1.0 / 12.0 ) * mass * (ey2 + ez2);
	r32 y = r32( 1.0 / 12.0 ) * mass * (ex2 + ez2);
	r32 z = r32( 1.0 / 12.0 ) * mass * (ex2 + ey2);
	q3Mat3 I = q3Diagonal( x, y, z );

	I = local.rotation * I * q3Transpose( local.rotation );
	q3Mat3 identity;
	q3Identity( identity );
	I += (identity * q3Dot( local.position, local.position ) - q3OuterProduct( local.position, local.position )) * mass;

	md->center = local.position;
	md->inertia = I;
	md->mass = mass;
}

//--------------------------------------------------------------------------------------------------
const i32 kBoxIndices[ 36 ] = {
	1 - 1, 7 - 1, 5 - 1,
	1 - 1, 3 - 1, 7 - 1,
	1 - 1, 4 - 1, 3 - 1,
	1 - 1, 2 - 1, 4 - 1,
	3 - 1, 8 - 1, 7 - 1,
	3 - 1, 4 - 1, 8 - 1,
	5 - 1, 7 - 1, 8 - 1,
	5 - 1, 8 - 1, 6 - 1,
	1 - 1, 5 - 1, 6 - 1,
	1 - 1, 6 - 1, 2 - 1,
	2 - 1, 6 - 1, 8 - 1,
	2 - 1, 8 - 1, 4 - 1
};

//--------------------------------------------------------------------------------------------------
void qWhatDoesThis::Render( const q3Transform& tx, bool awake, q3Render* render ) const
{
	q3Transform world = q3Mul( tx, local );

	q3Vec3 vertices[ 8 ] = {
		q3Vec3( -e.x, -e.y, -e.z ),
		q3Vec3( -e.x, -e.y,  e.z ),
		q3Vec3( -e.x,  e.y, -e.z ),
		q3Vec3( -e.x,  e.y,  e.z ),
		q3Vec3(  e.x, -e.y, -e.z ),
		q3Vec3(  e.x, -e.y,  e.z ),
		q3Vec3(  e.x,  e.y, -e.z ),
		q3Vec3(  e.x,  e.y,  e.z )
	};

    //this is a change


	for ( i32 i = 0; i < 36; i += 3 )
	{
		q3Vec3 a = q3Mul( world, vertices[ kBoxIndices[ i ] ] );
		q3Vec3 b = q3Mul( world, vertices[ kBoxIndices[ i + 1 ] ] );
		q3Vec3 c = q3Mul( world, vertices[ kBoxIndices[ i + 2 ] ] );

		q3Vec3 n = q3Normalize( q3Cross( b - a, c - a ) );

		render->SetTriNormal( n.x, n.y, n.z );
		render->Triangle( a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z );
	}
}