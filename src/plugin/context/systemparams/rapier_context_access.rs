use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use std::ops::{Deref, DerefMut};

use super::super::{DefaultRapierContext, RapierContext, RapierContextEntityLink};
/// Utility [`SystemParam`] to easily access the default world [`RapierContext`] immutably
#[derive(SystemParam)]
pub struct DefaultRapierContextAccess<'w, 's, T: Component = DefaultRapierContext> {
    rapier_context: Query<'w, 's, &'static RapierContext, With<T>>,
}

impl<'w, 's, T: Component> DefaultRapierContextAccess<'w, 's, T> {
    /// Use this method if you only have one world.
    pub fn single(&'_ self) -> &RapierContext {
        self.rapier_context.single()
    }
}

impl<'w, 's> Deref for DefaultRapierContextAccess<'w, 's> {
    type Target = RapierContext;

    fn deref(&self) -> &Self::Target {
        self.rapier_context.single()
    }
}

/// Utility [`SystemParam`] to easily access the default world [`RapierContext`] mutably
#[derive(SystemParam)]
pub struct DefaultRapierContextAccessMut<'w, 's, T: Component = DefaultRapierContext> {
    rapier_context: Query<'w, 's, &'static mut RapierContext, With<T>>,
}

impl<'w, 's, T: Component> Deref for DefaultRapierContextAccessMut<'w, 's, T> {
    type Target = RapierContext;

    fn deref(&self) -> &Self::Target {
        self.rapier_context.single()
    }
}

impl<'w, 's> DerefMut for DefaultRapierContextAccessMut<'w, 's> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // TODO: should we cache the result ?
        self.rapier_context.single_mut().into_inner()
    }
}

/// Utility [`SystemParam`] to easily access any [`RapierContext`] immutably
#[derive(SystemParam)]
pub struct RapierContextAccess<'w, 's> {
    /// Query used to retrieve a [`RapierContext`].
    /// It's helpful to iterate over every rapier contexts,
    /// or get a handle over a specific context, for example through:
    /// - a marker component such as [`DefaultRapierContext`]
    /// - a [`RapierContextEntityLink`]. See [context](RapierContextAccess::context)
    pub rapier_context: Query<'w, 's, &'static RapierContext>,
}

impl<'w, 's> RapierContextAccess<'w, 's> {
    /// Retrieves the rapier context responsible for the entity owning the given [`RapierContextEntityLink`].
    pub fn context(&self, link: RapierContextEntityLink) -> &'_ RapierContext {
        self.rapier_context
            .get(link.0)
            .expect("RapierContextEntityLink.0 refers to an entity without RapierContext.")
    }
}

impl<'w, 's> Deref for RapierContextAccess<'w, 's> {
    type Target = RapierContext;

    fn deref(&self) -> &Self::Target {
        self.rapier_context.single()
    }
}

/// Utility [`SystemParam`] to easily access any [`RapierContext`] mutably
#[derive(SystemParam)]
pub struct RapierContextAccessMut<'w, 's> {
    /// Query used to retrieve a [`RapierContext`].
    /// It's helpful to iterate over every rapier contexts,
    /// or get a handle over a specific context, for example through:
    /// - a marker component such as [`DefaultRapierContext`]
    /// - a [`RapierContextEntityLink`]. See [context](RapierContextAccess::context)
    pub rapier_context: Query<'w, 's, &'static mut RapierContext>,
}

impl<'w, 's> RapierContextAccessMut<'w, 's> {
    /// Retrieves the rapier context responsible for the entity owning the given [`RapierContextEntityLink`].
    pub fn context(&mut self, link: RapierContextEntityLink) -> &'_ mut RapierContext {
        self.rapier_context
            .get_mut(link.0)
            .expect("RapierContextEntityLink.0 refers to an entity without RapierContext.")
            .into_inner()
    }
}
